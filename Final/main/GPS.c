#include "GPS.h"

static const char *TAG = "GPS_TEST";

// Helper function to extract NMEA type (e.g., "GLL", "GSA") from a sentence
const char* get_nmea_type(const char* nmea) {
    // NMEA sentences start with $GPxxx, $GNxxx, etc.
    if (nmea[0] == '$' && strlen(nmea) > 6) {
        static char type[4];
        strncpy(type, nmea + 3, 3); // Copy 3 chars after "$GP" or "$GN"
        type[3] = '\0';
        return type;
    }
    return NULL;
}

// Helper function to process buffer and store NMEA sentences
void process_uart_buffer(char* buffer, nvs_handle_t nvs_handle) {
    char* ptr = buffer;
    while ((ptr = strchr(ptr, '$')) != NULL) { // Find next '$'
        char* end = strpbrk(ptr, "\r\n");      // Find end of sentence
        if (end) {
            size_t len = end - ptr;
            if (len > 6 && len < 100) { // Basic length check
                char nmea[101];
                strncpy(nmea, ptr, len);
                nmea[len] = '\0';

                // Extract type and store in NVS
                const char* type = get_nmea_type(nmea);
                if (type) {
                    esp_err_t ret = nvs_set_str(nvs_handle, type, nmea);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "Stored NMEA type %s: %s", type, nmea);
                        nvs_commit(nvs_handle);
                    } else {
                        ESP_LOGE(TAG, "Failed to store NMEA type %s", type);
                    }

                    // If sentence is GLL (contains lat/lon info), parse and store them
                    if (strcmp(type, "GLL") == 0) {
                        // Example: $GNGLL,3405.44176,S,01852.26002,E,150405.00,A,A*65
                        // Fields: 1=lat, 2=N/S, 3=lon, 4=E/W
                        char lat[16] = {0};
                        char lon[16] = {0};
                        char ns = 'N', ew = 'E';
                        // Tokenize the sentence
                        char sentence_copy[101];
                        strncpy(sentence_copy, nmea, 100);
                        sentence_copy[100] = '\0';
                        char* saveptr;
                        int field = 0;
                        char* token = strtok_r(sentence_copy, ",", &saveptr); // $GNGLL (field 0)
                        while (token != NULL) {
                            field++;
                            token = strtok_r(NULL, ",", &saveptr);
                            if (field == 1 && token) {
                                // Latitude field
                                size_t lat_len = strcspn(token, ",\r\n");
                                if (lat_len > 15) lat_len = 15;
                                strncpy(lat, token, lat_len);
                                lat[lat_len] = '\0';
                            }
                            if (field == 2 && token) {
                                ns = token[0];
                            }
                            if (field == 3 && token) {
                                // Longitude field
                                size_t lon_len = strcspn(token, ",\r\n");
                                if (lon_len > 15) lon_len = 15;
                                strncpy(lon, token, lon_len);
                                lon[lon_len] = '\0';
                            }
                            if (field == 4 && token) {
                                ew = token[0];
                            }
                        }
                        // Compose lat/lon with direction
                        char lat_full[18], lon_full[18];
                        snprintf(lat_full, sizeof(lat_full), "%s,%c", lat, ns);
                        snprintf(lon_full, sizeof(lon_full), "%s,%c", lon, ew);
                        // Store in NVS under keys "LAT" and "LON"
                        ret = nvs_set_str(nvs_handle, "LAT", lat_full);
                        if (ret == ESP_OK) {
                            ESP_LOGI(TAG, "Stored LAT: %s", lat_full);
                            nvs_commit(nvs_handle);
                        } else {
                            ESP_LOGE(TAG, "Failed to store LAT");
                        }
                        ret = nvs_set_str(nvs_handle, "LON", lon_full);
                        if (ret == ESP_OK) {
                            ESP_LOGI(TAG, "Stored LON: %s", lon_full);
                            nvs_commit(nvs_handle);
                        } else {
                            ESP_LOGE(TAG, "Failed to store LON");
                        }
                    }
                }
            }
            ptr = end + 1; // Move to next possible sentence
        } else {
            break; // No complete sentence found
        }
    }
}

int get_gps_lat_lon(float *latitude, float *longitude) {
    // Validate input parameters
    if (!latitude || !longitude) {
        ESP_LOGE(TAG, "Invalid parameters: latitude and longitude pointers cannot be NULL");
        return -1;
    }
    
    esp_err_t ret;
    nvs_handle_t nvs_handle;
    ret = nvs_open("nmea_store", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle for reading!");
        return -1;
    }
    
    // Read latitude string from NVS
    char lat_buf[MAX_FULL_COORD_LEN + 1];
    size_t lat_buf_size = sizeof(lat_buf);
    ret = nvs_get_str(nvs_handle, "LAT", lat_buf, &lat_buf_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LAT from NVS");
        nvs_close(nvs_handle);
        return -1;
    }
    
    // Read longitude string from NVS
    char lon_buf[MAX_FULL_COORD_LEN + 1];
    size_t lon_buf_size = sizeof(lon_buf);
    ret = nvs_get_str(nvs_handle, "LON", lon_buf, &lon_buf_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LON from NVS");
        nvs_close(nvs_handle);
        return -1;
    }
    
    nvs_close(nvs_handle);
    
    // Parse latitude string (format: "DDMM.MMMMM,N/S")
    char *lat_comma = strchr(lat_buf, ',');
    if (!lat_comma || lat_comma == lat_buf) {
        ESP_LOGE(TAG, "Invalid latitude format: %s", lat_buf);
        return -1;
    }
    
    *lat_comma = '\0'; // Split the string
    char lat_direction = lat_comma[1];
    *latitude = nmea_to_decimal(lat_buf, lat_direction);
    
    // Parse longitude string (format: "DDDMM.MMMMM,E/W")
    char *lon_comma = strchr(lon_buf, ',');
    if (!lon_comma || lon_comma == lon_buf) {
        ESP_LOGE(TAG, "Invalid longitude format: %s", lon_buf);
        return -1;
    }
    
    *lon_comma = '\0'; // Split the string
    char lon_direction = lon_comma[1];
    *longitude = nmea_to_decimal(lon_buf, lon_direction);
    
    ESP_LOGI(TAG, "GPS coordinates retrieved: LAT=%.6f, LON=%.6f", *latitude, *longitude);
    return 0; // Success
}

double nmea_to_decimal(const char* nmea_coord, char direction) {
    if (!nmea_coord || strlen(nmea_coord) == 0) {
        ESP_LOGE(TAG, "Invalid NMEA coordinate");
        return 0.0;
    }
    
    // Parse NMEA coordinate format: DDMM.MMMMM or DDDMM.MMMMM
    double coord = atof(nmea_coord);
    
    // Extract degrees and minutes
    int degrees;
    double minutes;
    
    if (strlen(nmea_coord) > 7) { // Longitude format DDDMM.MMMMM
        degrees = (int)(coord / 100);
        minutes = coord - (degrees * 100);
    } else { // Latitude format DDMM.MMMMM
        degrees = (int)(coord / 100);
        minutes = coord - (degrees * 100);
    }
    
    // Convert to decimal degrees
    double decimal = degrees + (minutes / 60.0);
    
    // Apply direction (negative for South and West)
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

int gps_uart_init(void){
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,         // 8 data bits
        .parity    = UART_PARITY_DISABLE,      // No parity
        .stop_bits = UART_STOP_BITS_1,         // 1 stop bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // No hardware flow control
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters
    esp_err_t err = uart_param_config(UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters");
        return -1;
    }

    // Set UART pins (TX, RX, RTS, CTS)
    err = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins");
        return -1;
    }

    // Install UART driver with RX buffer
    err = uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return -1;
    }
    ESP_LOGI(TAG, "GPS UART initialized on UART2");
    return 0;

}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Open NVS handle in read-write mode
    nvs_handle_t nvs_handle;
    ret = nvs_open("nmea_store", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        return;
    }

    gps_uart_init();
    
    uint8_t data[UART_BUF_SIZE];

    ESP_LOGI(TAG, "GPS UART test started. Waiting for NMEA sentences...");

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate
            ESP_LOGI(TAG, "Received buffer:\n%s", (char *)data);

            // Process buffer for multiple NMEA sentences
            process_uart_buffer((char *)data, nvs_handle);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    nvs_close(nvs_handle);
}
