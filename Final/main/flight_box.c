#include "flight_box.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *FLIGHT_TAG = "FLIGHT_ZONE";

// NVS namespace for flight zone data
#define FLIGHT_ZONE_NAMESPACE "flight_zone"

// Initialize NVS for flight zone storage
int flight_zone_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(FLIGHT_TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(FLIGHT_TAG, "Failed to initialize in flight_zone NVS");
        return -1;
    }

    ESP_LOGI(FLIGHT_TAG, "Flight zone NVS initialized");
    return 0;
}


// Store polygon flight zone boundary
int store_polygon_flight_zone(coordinate_t *boundary_points, int num_points) {
    if (num_points > MAX_BOUNDARY_POINTS || num_points < 3) {
        ESP_LOGE(FLIGHT_TAG, "Invalid number of boundary points: %d (must be 3-%d)", 
                 num_points, MAX_BOUNDARY_POINTS);
        return -1;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    // Open NVS handle
    ret = nvs_open(FLIGHT_ZONE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(FLIGHT_TAG, "Error opening NVS handle for flight zone storage");
        return -1;
    }
    
    // Create flight zone structure
    flight_zone_t zone;
    zone.zone_type = 1; // Polygon
    zone.point_count = num_points;
    zone.radius_km = 0.0; // Not used for polygon
    
    // Copy boundary points
    for (int i = 0; i < num_points; i++) {
        zone.points[i] = boundary_points[i];
    }
    
    // Calculate centroid for reference
    double sum_lat = 0.0, sum_lon = 0.0;
    for (int i = 0; i < num_points; i++) {
        sum_lat += boundary_points[i].latitude;
        sum_lon += boundary_points[i].longitude;
    }
    zone.center_lat = sum_lat / num_points;
    zone.center_lon = sum_lon / num_points;
    
    // Store flight zone data
    ret = nvs_set_blob(nvs_handle, "zone_data", &zone, sizeof(flight_zone_t));
    if (ret != ESP_OK) {
        ESP_LOGE(FLIGHT_TAG, "Failed to store flight zone data");
        nvs_close(nvs_handle);
        return -1;
    }
    
    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(FLIGHT_TAG, "Failed to commit flight zone data");
        nvs_close(nvs_handle);
        return -1;
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(FLIGHT_TAG, "Polygon flight zone stored with %d points, Centroid(%.6f, %.6f)", 
             num_points, zone.center_lat, zone.center_lon);
    
    // Log all boundary points
    for (int i = 0; i < num_points; i++) {
        ESP_LOGI(FLIGHT_TAG, "Point %d: (%.6f, %.6f)", i+1, 
                 boundary_points[i].latitude, boundary_points[i].longitude);
    }
    
    return 0;
}

// Retrieve stored flight zone boundary
int get_flight_zone(flight_zone_t *zone) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    // Open NVS handle in read-only mode
    ret = nvs_open(FLIGHT_ZONE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(FLIGHT_TAG, "Error opening NVS handle for flight zone reading");
        return -1;
    }
    
    // Get the size of stored data
    size_t required_size = sizeof(flight_zone_t);
    ret = nvs_get_blob(nvs_handle, "zone_data", zone, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGE(FLIGHT_TAG, "Failed to retrieve flight zone data");
        nvs_close(nvs_handle);
        return -1;
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(FLIGHT_TAG, "Flight zone retrieved: Type=%s, Center(%.6f, %.6f)", 
             (zone->zone_type == 0) ? "Circular" : "Polygon",
             zone->center_lat, zone->center_lon);
    
    return 0;
}

// Calculate distance between two coordinates using Haversine formula
double calculate_distance_km(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0; // Earth's radius in km
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dlat/2) * sin(dlat/2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

// Check if point is inside circular zone
int is_inside_circular_zone(const flight_zone_t *zone, double lat, double lon) {
    double distance = calculate_distance_km(zone->center_lat, zone->center_lon, lat, lon);
    return (distance <= zone->radius_km) ? 1 : 0;
}

// Check if point is inside polygon using ray casting algorithm
int is_inside_polygon_zone(const flight_zone_t *zone, double lat, double lon) {
    int inside = 0;
    int j = zone->point_count - 1;
    
    for (int i = 0; i < zone->point_count; i++) {
        double xi = zone->points[i].longitude;
        double yi = zone->points[i].latitude;
        double xj = zone->points[j].longitude;
        double yj = zone->points[j].latitude;
        
        if (((yi > lat) != (yj > lat)) && 
            (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

// Main function to check if balloon is within flight zone
int is_within_flight_zone(double balloon_lat, double balloon_lon) {
    flight_zone_t zone;
    
    // Get stored flight zone
    if (get_flight_zone(&zone) != 0) {
        ESP_LOGE(FLIGHT_TAG, "No flight zone configured");
        return -1; // No zone configured
    }
    
    int inside;
    if (zone.zone_type == 0) {
        // Circular zone
        inside = is_inside_circular_zone(&zone, balloon_lat, balloon_lon);
        double distance = calculate_distance_km(zone.center_lat, zone.center_lon, 
                                              balloon_lat, balloon_lon);
        ESP_LOGI(FLIGHT_TAG, "Distance from center: %.2f km (limit: %.2f km)", 
                 distance, zone.radius_km);
    } else {
        // Polygon zone  
        inside = is_inside_polygon_zone(&zone, balloon_lat, balloon_lon);
    }
    
    ESP_LOGI(FLIGHT_TAG, "Balloon position (%.6f, %.6f) is %s flight zone", 
             balloon_lat, balloon_lon, inside ? "WITHIN" : "OUTSIDE");
    
    return inside;
}

// Clear stored flight zone
int clear_flight_zone(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(FLIGHT_ZONE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(FLIGHT_TAG, "Error opening NVS handle for clearing flight zone");
        return -1;
    }
    
    ret = nvs_erase_key(nvs_handle, "zone_data");
    if (ret != ESP_OK) {
        ESP_LOGE(FLIGHT_TAG, "Failed to clear flight zone data");
        nvs_close(nvs_handle);
        return -1;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(FLIGHT_TAG, "Flight zone cleared");
    return 0;
}

// Example usage functions for testing
int setup_example_circular_flight_zone(void) {
    // Example: Set up a circular flight zone centered at Cape Town
    // Coordinates: -33.9249° S, 18.4241° E, Radius: 10 km
    double center_lat = -33.9249;
    double center_lon = 18.4241;
    double radius = 10.0; // 10 km radius
    
    if (store_circular_flight_zone(center_lat, center_lon, radius) == 0) {
        ESP_LOGI(FLIGHT_TAG, "Example circular flight zone configured successfully");
    }
}

void setup_example_polygon_flight_zone(void) {
    // Example: Set up a rectangular polygon flight zone
    coordinate_t boundary[] = {
        {-33.9000, 18.4000},  // Northwest corner
        {-33.9000, 18.4500},  // Northeast corner  
        {-33.9500, 18.4500},  // Southeast corner
        {-33.9500, 18.4000}   // Southwest corner
    };
    
    int num_points = sizeof(boundary) / sizeof(coordinate_t);
    
    if (store_polygon_flight_zone(boundary, num_points) == 0) {
        ESP_LOGI(FLIGHT_TAG, "Example polygon flight zone configured successfully");
    }
}

int setup_box_flight_zone(coordinate_t[] boundary) {

    if (store_polygon_flight_zone(boundary, 4) == 0) {
        ESP_LOGI(FLIGHT_TAG, "Example polygon flight zone configured successfully");
        return 0;
    }
    else {
        return -1;
    }
}