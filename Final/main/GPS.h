#ifndef GPS_H
#define GPS_H

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- GPS UART Configuration Constants ---
#define UART_NUM            UART_NUM_2         // Use UART2 for GPS communication
#define UART_TX_PIN         (17)               // ESP32 TX pin connected to GPS RX
#define UART_RX_PIN         (16)               // ESP32 RX pin connected to GPS TX
#define UART_BAUD_RATE      (9600)             // Default baud rate for MAX-M10S GPS
#define UART_BUF_SIZE       (1024)             // Buffer size for incoming NMEA data

// --- NVS Storage Configuration ---
#define NVS_NAMESPACE       "nmea_store"       // NVS namespace for GPS data storage

// --- NMEA Sentence Parsing Constants ---
#define MAX_NMEA_LENGTH     (100)              // Maximum length for NMEA sentence
#define MAX_COORDINATE_LEN  (16)               // Maximum length for coordinate string
#define MAX_FULL_COORD_LEN  (18)               // Maximum length for coordinate with direction

// --- Function Prototypes ---

/**
 * @brief Extract NMEA sentence type from raw NMEA data
 * 
 * Parses NMEA sentences to extract the 3-character type identifier
 * (e.g., "GLL", "GSA", "RMC") from sentences like "$GPGLL" or "$GNGLL".
 * 
 * @param nmea Pointer to NMEA sentence string
 * @return Pointer to static string containing NMEA type, or NULL if invalid
 */
const char* get_nmea_type(const char* nmea);

/**
 * @brief Process UART buffer containing multiple NMEA sentences
 * 
 * Parses incoming UART data buffer for complete NMEA sentences,
 * extracts GPS data, and stores it in NVS flash memory.
 * Handles multiple sentences in a single buffer and stores:
 * - Individual NMEA sentences by type
 * - Parsed latitude and longitude from GLL sentences
 * 
 * @param buffer Pointer to UART data buffer
 * @param nvs_handle Open NVS handle for data storage
 */
void process_uart_buffer(char* buffer, nvs_handle_t nvs_handle);

/**
 * @brief Retrieve stored GPS coordinates from NVS
 * 
 * Reads the most recently stored latitude and longitude coordinates
 * from NVS flash memory and converts them to decimal degrees format.
 * 
 * @param latitude Pointer to float variable to store latitude in decimal degrees
 * @param longitude Pointer to float variable to store longitude in decimal degrees
 * @return 0 on success, -1 on failure
 * 
 * @example
 * float lat, lon;
 * if (get_gps_lat_lon(&lat, &lon) == 0) {
 *     printf("Location: %.6f, %.6f\n", lat, lon);
 * }
 */
int get_gps_lat_lon(float *latitude, float *longitude);

/**
 * @brief Initialize GPS UART communication
 * 
 * Configures UART2 for GPS communication with proper settings:
 * - 9600 baud rate
 * - 8 data bits, no parity, 1 stop bit
 * - No hardware flow control
 * - Specified TX/RX pins
 * 
 * @return 0 on success, negative value on failure
 */
int gps_uart_init(void);

/**
 * @brief Initialize GPS NVS storage
 * 
 * Initializes NVS flash and opens handle for GPS data storage.
 * Creates the "nmea_store" namespace if it doesn't exist.
 * 
 * @param nvs_handle Pointer to store opened NVS handle
 * @return 0 on success, negative value on failure
 */
int gps_nvs_init(nvs_handle_t *nvs_handle);

/**
 * @brief Main GPS processing task
 * 
 * Continuous task that reads NMEA data from GPS module via UART,
 * processes the data, and stores coordinates in NVS flash.
 * This function runs indefinitely and should be called as a FreeRTOS task.
 * 
 * @param pvParameter Task parameter (unused)
 */
void gps_task(void *pvParameter);

/**
 * @brief Parse GLL sentence for coordinates
 * 
 * Extracts latitude and longitude from NMEA GLL sentences.
 * GLL format: $GNGLL,lat,N/S,lon,E/W,time,status,checksum
 * 
 * @param gll_sentence Complete GLL NMEA sentence
 * @param lat_buf Buffer to store parsed latitude
 * @param lat_size Size of latitude buffer
 * @param lon_buf Buffer to store parsed longitude  
 * @param lon_size Size of longitude buffer
 * @return 0 on success, -1 on parse failure
 */
int parse_gll_sentence(const char* gll_sentence, char* lat_buf, size_t lat_size, 
                       char* lon_buf, size_t lon_size);

/**
 * @brief Convert NMEA coordinate to decimal degrees
 * 
 * Converts NMEA coordinate format (DDMM.MMMMM or DDDMM.MMMMM) 
 * to decimal degrees format for easier calculations.
 * 
 * @param nmea_coord NMEA coordinate string (e.g., "3405.44176")
 * @param direction Direction character ('N', 'S', 'E', 'W')
 * @return Decimal degrees (negative for South/West)
 */
double nmea_to_decimal(const char* nmea_coord, char direction);

/**
 * @brief Check if GPS has valid fix
 * 
 * Determines if GPS has acquired a valid position fix by
 * checking the most recent stored coordinate data.
 * 
 * @return 1 if valid fix available, 0 otherwise
 */
int gps_has_fix(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_H