#ifndef FLIGHTBOX_H
#define FLIGHTBOX_H

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of boundary points for polygon zones
#define MAX_BOUNDARY_POINTS 20

// Structure to hold a coordinate point (only define if not already defined)
#ifndef COORDINATE_T_DEFINED
#define COORDINATE_T_DEFINED
typedef struct {
    double latitude;
    double longitude;
} coordinate_t;
#endif

#ifndef FLIGHT_ZONE_T_DEFINED
#define FLIGHT_ZONE_T_DEFINED
typedef struct {
    coordinate_t points[MAX_BOUNDARY_POINTS];
    int point_count;
    double center_lat;
    double center_lon;
    double radius_km;  // For circular zones
    int zone_type;     // 0 = circular, 1 = polygon
} flight_zone_t;
#endif

// --- Function Prototypes ---

/**
 * @brief Initialize NVS for flight zone storage
 * 
 * Initializes the NVS (Non-Volatile Storage) system for storing
 * flight zone boundary data persistently in flash memory.
 * 
 * @return 0 on success, negative value on failure
 */
int flight_zone_init(void);

/**
 * @brief Store circular flight zone boundary
 * 
 * Defines and stores a circular flight zone with center coordinates
 * and radius. The balloon must stay within this circular area.
 * 
 * @param center_lat Center latitude in decimal degrees
 * @param center_lon Center longitude in decimal degrees  
 * @param radius_km Radius of allowed zone in kilometers
 * @return 0 on success, -1 on failure
 * 
 * @example
 * // Set 10km radius zone centered at Cape Town
 * store_circular_flight_zone(-33.9249, 18.4241, 10.0);
 */
int store_circular_flight_zone(double center_lat, double center_lon, double radius_km);

/**
 * @brief Store polygon flight zone boundary
 * 
 * Defines and stores a polygonal flight zone using multiple boundary points.
 * The balloon must stay within the area defined by these points.
 * 
 * @param boundary_points Array of coordinate points defining the polygon
 * @param num_points Number of points in the boundary (3 to MAX_BOUNDARY_POINTS)
 * @return 0 on success, -1 on failure
 * 
 * @example
 * coordinate_t boundary[] = {
 *     {-33.9000, 18.4000},
 *     {-33.9000, 18.4500}, 
 *     {-33.9500, 18.4500},
 *     {-33.9500, 18.4000}
 * };
 * store_polygon_flight_zone(boundary, 4);
 */
int store_polygon_flight_zone(coordinate_t *boundary_points, int num_points);

/**
 * @brief Retrieve stored flight zone boundary
 * 
 * Reads the currently configured flight zone from flash memory.
 * 
 * @param zone Pointer to flight_zone_t structure to store retrieved data
 * @return 0 on success, -1 on failure
 */
int get_flight_zone(flight_zone_t *zone);

/**
 * @brief Calculate distance between two coordinates
 * 
 * Uses the Haversine formula to calculate the great-circle distance
 * between two points on Earth's surface.
 * 
 * @param lat1 Latitude of first point in decimal degrees
 * @param lon1 Longitude of first point in decimal degrees
 * @param lat2 Latitude of second point in decimal degrees
 * @param lon2 Longitude of second point in decimal degrees
 * @return Distance in kilometers
 */
double calculate_distance_km(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Check if balloon is within configured flight zone
 * 
 * Main validation function that checks if the current balloon position
 * is within the stored flight zone boundaries.
 * 
 * @param balloon_lat Current balloon latitude in decimal degrees
 * @param balloon_lon Current balloon longitude in decimal degrees
 * @return 1 if within zone, 0 if outside zone, -1 if no zone configured
 * 
 * @example
 * double current_lat = -33.9200;
 * double current_lon = 18.4300;
 * 
 * int status = is_within_flight_zone(current_lat, current_lon);
 * if (status == 1) {
 *     ESP_LOGI("BALLOON", "Within flight zone - OK");
 * } else if (status == 0) {
 *     ESP_LOGW("BALLOON", "OUTSIDE flight zone - Take action!");
 * }
 */
int is_within_flight_zone(double balloon_lat, double balloon_lon);

/**
 * @brief Check if point is inside circular zone
 * 
 * @param zone Pointer to flight zone structure
 * @param lat Latitude to check
 * @param lon Longitude to check
 * @return 1 if inside, 0 if outside
 */
int is_inside_circular_zone(const flight_zone_t *zone, double lat, double lon);

/**
 * @brief Check if point is inside polygon zone
 * 
 * Uses ray casting algorithm to determine if a point is inside
 * a polygon defined by boundary points.
 * 
 * @param zone Pointer to flight zone structure  
 * @param lat Latitude to check
 * @param lon Longitude to check
 * @return 1 if inside, 0 if outside
 */
int is_inside_polygon_zone(const flight_zone_t *zone, double lat, double lon);

/**
 * @brief Clear stored flight zone
 * 
 * Removes the currently configured flight zone from flash memory.
 * 
 * @return 0 on success, -1 on failure
 */
int clear_flight_zone(void);

/**
 * @brief Set up example circular flight zone
 * 
 * Configures a sample circular flight zone for testing purposes.
 * Creates a 10km radius zone centered at Cape Town coordinates.
 */
void setup_example_circular_flight_zone(void);

/**
 * @brief Set up example polygon flight zone
 * 
 * Configures a sample rectangular polygon flight zone for testing purposes.
 * Creates a rectangular boundary around Cape Town area.
 */
void setup_example_polygon_flight_zone(void);

int setup_box_flight_zone(coordinate_t[] boundary);

#ifdef __cplusplus
}
#endif

#endif // FLIGHTBOX_H
