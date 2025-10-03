#ifndef SETUP_H
#define SETUP_H

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

// Structure to hold flight zone boundary
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

int flight_zone_init(void);
int store_circular_flight_zone(double center_lat, double center_lon, double radius_km);
int store_polygon_flight_zone(coordinate_t *boundary_points, int num_points);
int get_flight_zone(flight_zone_t *zone);
double calculate_distance_km(double lat1, double lon1, double lat2, double lon2);
int is_within_flight_zone(double balloon_lat, double balloon_lon);
int is_inside_circular_zone(const flight_zone_t *zone, double lat, double lon);
int is_inside_polygon_zone(const flight_zone_t *zone, double lat, double lon);
int clear_flight_zone(void);
void setup_example_circular_zone(void);
void setup_example_polygon_zone(void);

#ifdef __cplusplus
}
#endif

#endif // SETUP_H