#include "landzone.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *LANDING_TAG = "LANDING_ZONE";


#define LANDING_ZONE_NAMESPACE "landing_zone"

// Initialize NVS for landing zone storage
int landing_zone_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(LANDING_TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(LANDING_TAG, "Failed to initialize in landing_zone NVS");
        return -1;
    }
        ESP_LOGI(LANDING_TAG, "Landing zone NVS initialized");
    return 0;
}

// Store circular landing zone boundary
int store_landing_zone(landing_zone_t zone) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    // Open NVS handle
    ret = nvs_open(LANDING_ZONE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(LANDING_TAG, "Error opening NVS handle for landing zone storage");
        return -1;
    }
    
    // Store landing zone data
    ret = nvs_set_blob(nvs_handle, "zone_data", &zone, sizeof(landing_zone_t));
    if (ret != ESP_OK) {
        ESP_LOGE(LANDING_TAG, "Failed to store landing zone data");
        nvs_close(nvs_handle);
        return -1;
    }
    
    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(LANDING_TAG, "Failed to commit landing zone data");
        nvs_close(nvs_handle);
        return -1;
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(LANDING_TAG, "Landing zone stored: Center(%.6f, %.6f), Radius=%.2f km", 
             center_lat, center_lon, radius_km);
    return 0;
}



// Retrieve stored landing zone boundary
int get_landing_zone(landing_zone_t *zone) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    // Open NVS handle in read-only mode
    ret = nvs_open(LANDING_ZONE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(LANDING_TAG, "Error opening NVS handle for landing zone reading");
        return -1;
    }
    
    // Get the size of stored data
    size_t required_size = sizeof(landing_zone_t);
    ret = nvs_get_blob(nvs_handle, "zone_data", zone, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGE(LANDING_TAG, "Failed to retrieve landing zone data");
        nvs_close(nvs_handle);
        return -1;
    }
    
    nvs_close(nvs_handle);

    ESP_LOGI(LANDING_TAG, "Landing zone retrieved: Center(%.6f, %.6f), Radius=%.2f km",
             zone->center_lat, zone->center_lon, zone->radius_km);

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

// Check if point is inside zone
int is_inside_zone(const landing_zone_t *zone, double lat, double lon) {
    double distance = calculate_distance_km(zone->center_lat, zone->center_lon, lat, lon);
    return (distance <= zone->radius_km) ? 1 : 0;
}

// Clear stored landing zone
int clear_landing_zone(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(LANDING_ZONE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(LANDING_TAG, "Error opening NVS handle for clearing landing zone");
        return -1;
    }
    
    ret = nvs_erase_key(nvs_handle, "zone_data");
    if (ret != ESP_OK) {
        ESP_LOGE(LANDING_TAG, "Failed to clear landing zone data");
        nvs_close(nvs_handle);
        return -1;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    ESP_LOGI(LANDING_TAG, "Landing zone cleared");
    return 0;
}

// Example usage functions for testing
void setup_example_landing_zone(void) {
    // Example: Set up a circular landing zone centered at Cape Town
    // Coordinates: -33.9249° S, 18.4241° E, Radius: 10 km
    double center_lat = -33.9249;
    double center_lon = 18.4241;
    double radius = 10.0; // 10 km radius

    if (store_landing_zone(center_lat, center_lon, radius) == 0) {
        ESP_LOGI(LANDING_TAG, "Example circular landing zone configured successfully");
    }
}
