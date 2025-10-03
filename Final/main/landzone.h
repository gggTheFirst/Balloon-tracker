#ifndef LANDZONE_H
#define LANDZONE_H

#ifdef __cplusplus
extern "C" {
#endif

// Structure to hold a coordinate point (only define if not already defined)
#ifndef COORDINATE_T_DEFINED
#define COORDINATE_T_DEFINED
typedef struct {
    double latitude;
    double longitude;
} coordinate_t;
#endif


typedef struct {
    coordinate_t center;
    double radius_km;
    double height_m; //  max height for landing zone
} landing_zone_t;


int landing_zone_init(void);

int store_landing_zone(landing_zone_t zone);

int get_landing_zone(landing_zone_t *zone);

double calculate_distance_km(double lat1, double lon1, double lat2, double lon2);

int is_within_flight_zone(double balloon_lat, double balloon_lon);

int is_inside_zone(const landing_zone_t *zone, double lat, double lon);

int clear_landing_zone(void);

void setup_example_landing_zone(void);

int store_landing_zone(landing_zone_t zone);

#ifdef __cplusplus
}
#endif

#endif // FLIGHTBOX_H
