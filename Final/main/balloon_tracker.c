#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"

// Include all module headers
#include "motors.h"
#include "temp_sensor.h"
#include "imu.h"
#include "GPS.h"
#include "setup.h"
#include "flight_box.h"
#include "landzone.h"

//prototypes
int end_zone_check(void);
int check_parachute_deploy(void);
// Balloon states
typedef enum {
    STATE_INITIALIZATION,
    STATE_SETUP,
    STATE_ASCENT,
    STATE_FLIGHT,
    STATE_DESCENT,
    STATE_FLOATING, 
    STATE_LANDED
} balloon_state_t;

// Measurement data structure (enhanced)
typedef struct {
    // Temperature sensors
    float inside_temperature;
    float outside_temperature;
    
    // Barometric data
    float pressure_hpa;
    float altitude_m;
    
    // GPS data
    double latitude;
    double longitude;
    bool gps_valid;
    
    // IMU data
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    
    // Flight status
    balloon_state_t state;
    bool within_flight_zone;
    bool parachute_deployed;
    bool balloon_released;
    float max_altitude_m; // For descent rate calculation
    
} measurement_data_t;

static const char *TAG = "BALLOON_TRACKER";
flight_zone_t current_flight_zone;
landing_zone_t current_landing_zone;
static measurement_data_t sensor_data;
static balloon_state_t current_state = STATE_INITIALIZATION;

void measure_data(measurement_data_t *data, uint8_t display_info){
    data->inside_temperature = measure_inside_temperature();
    data->outside_temperature = measure_outside_temperature();

    mpu9250_data_t mpu_data;
    bmp280_data_t bmp_data;

    if (read_mpu9250_data(&mpu_data) == ESP_OK) {
        data->accel_x = (mpu_data.accel_x);
        data->accel_y = (mpu_data.accel_y);
        data->accel_z = (mpu_data.accel_z);
        data->gyro_x = (mpu_data.gyro_x);
        data->gyro_y = (mpu_data.gyro_y);
        data->gyro_z = (mpu_data.gyro_z);
    } else {
        ESP_LOGW(TAG, "Failed to read MPU9250 data");
        data->accel_x = data->accel_y = data->accel_z = 0.0;
        data->gyro_x = data->gyro_y = data->gyro_z = 0.0;
    }

    if (read_bmp280_data(&bmp_data) == ESP_OK) {
        data->pressure_hpa = bmp_data.pressure_hpa;
        data->altitude_m = calculate_altitude(bmp_data.pressure_hpa);
        if (data->altitude_m > data->max_altitude_m) {
            data->max_altitude_m = data->altitude_m;
            ESP_LOGI(TAG, "New maximum altitude: %.1f m", data->max_altitude_m);
        }
    } else {
        ESP_LOGW(TAG, "Failed to read BMP280 data");
        data->pressure_hpa = 0.0;
        data->altitude_m = 0.0;

        
    }

    // Get GPS coordinates using the updated function
    float latitude, longitude;
    if (get_gps_lat_lon(&latitude, &longitude) == 0) {
        data->latitude = (double)latitude;
        data->longitude = (double)longitude;
        data->gps_valid = true;
    } else {
        data->latitude = 0.0;
        data->longitude = 0.0;
        data->gps_valid = false;
    }

            // Update flight status
        sensor_data.state = current_state;
        
        // Check flight zone compliance
        if (sensor_data.gps_valid) {
            sensor_data.within_flight_zone = is_within_flight_zone(sensor_data.latitude, sensor_data.longitude);
            
            if (!sensor_data.within_flight_zone) {
                ESP_LOGW(TAG, "ALERT: Balloon is outside flight zone!");
                // Could trigger emergency procedures here
            }
        } else {
            ESP_LOGW(TAG, "GPS fix not available - cannot check flight zone");
            sensor_data.within_flight_zone = false;
        }
        
        // Log current measurements
        if (display_info) {
            ESP_LOGI(TAG, "Current Measurements:");
        
        ESP_LOGI("MEASUREMENTS", "  Altitude: %.1f m", sensor_data.altitude_m);
        ESP_LOGI("MEASUREMENTS", "  Pressure: %.2f hPa", sensor_data.pressure_hpa);
        ESP_LOGI("MEASUREMENTS", "  Inside Temp: %.2f°C", sensor_data.inside_temperature);
        ESP_LOGI("MEASUREMENTS", "  Outside Temp: %.2f°C", sensor_data.outside_temperature);
        
        if (sensor_data.gps_valid) {
            ESP_LOGI("MEASUREMENTS", "  GPS: %.6f, %.6f", sensor_data.latitude, sensor_data.longitude);
            ESP_LOGI("MEASUREMENTS", "  Flight Zone: %s", sensor_data.within_flight_zone ? "OK" : "VIOLATION");
        } else {
            ESP_LOGI("MEASUREMENTS", "  GPS: No fix available");
        }
        
        ESP_LOGI("MEASUREMENTS", "  Acceleration: X=%.3f, Y=%.3f, Z=%.3f g", 
                 sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        ESP_LOGI("MEASUREMENTS", "  Gyroscope: X=%.3f, Y=%.3f, Z=%.3f °/s", 
                 sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        }
        
}

int transmit_data(void) {
    // Transmit sensor data to ground station
    ESP_LOGI(TAG, "=== Transmitting to ground station ===");
        
        // Prepare telemetry packet
        char telemetry_packet[512];
        int packet_length = snprintf(telemetry_packet, sizeof(telemetry_packet),
            "$BALLOON,%.6f,%.6f,%.1f,%.2f,%.2f,%.2f,%d,%d,%d*\r\n",
            sensor_data.gps_valid ? sensor_data.latitude : 0.0,
            sensor_data.gps_valid ? sensor_data.longitude : 0.0,
            sensor_data.altitude_m,
            sensor_data.pressure_hpa,
            sensor_data.inside_temperature,
            sensor_data.outside_temperature,
            sensor_data.gps_valid ? 1 : 0,
            sensor_data.within_flight_zone ? 1 : 0,
            (int)sensor_data.state
            
        );
        
        // Send via UART to ground station
        if (packet_length > 0 && packet_length < sizeof(telemetry_packet)) {
            uart_write_bytes(UART_NUM_1, telemetry_packet, packet_length);
            ESP_LOGI(TAG, "Telemetry sent: %s", telemetry_packet);
        } else {
            ESP_LOGE(TAG, "Failed to format telemetry packet");
        }
        
        // Also send detailed sensor data
        char detailed_packet[256];
        int detailed_length = snprintf(detailed_packet, sizeof(detailed_packet),
            "$SENSORS,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d*\r\n",
            sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
            sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z,
            sensor_data.parachute_deployed ? 1 : 0,
            sensor_data.balloon_released ? 1 : 0
        );
        
        if (detailed_length > 0 && detailed_length < sizeof(detailed_packet)) {
            uart_write_bytes(UART_NUM_1, detailed_packet, detailed_length);
            ESP_LOGI(TAG, "Sensor data sent: %s", detailed_packet);
        }
    return 0;
}

int ground_station_uart_init(void) {
    // Configure UART for ground station communication
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    esp_err_t err;

 
    // Configure UART1 for ground station communication (different from GPS UART2)
    err = uart_param_config(UART_NUM_1, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters");
        return -1;
    }
    
    err = uart_set_pin(UART_NUM_1, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // TX=4, RX=5
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pin");
        return -1;
    }

    err = uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return -1;
    }

    ESP_LOGI(TAG, "Ground station UART initialized on UART1");
    return 0;
}

int system_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        return -1;
    }

    if (ground_station_uart_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize ground station UART");
        return -1;
    }

    // Initialize peripherals
    if (gps_uart_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize GPS UART");
        return -1;
    }
    if (ground_station_uart_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize ground station UART");
        return -1;
    }
    if (temp_sensor_configure_adc() != 0) {
        ESP_LOGE(TAG, "Failed to configure temperature sensors");
        return -1;
    }

    if (imu_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize IMU sensors");
        return -1;
    }
    if (configure_servos() != 0) {
        ESP_LOGE(TAG, "Failed to configure servos");
        return -1;
    }
    if (flight_zone_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize flight zone");
        return -1;
    }
    if (landing_zone_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize landing zone");
        return -1;
    }
    ESP_LOGI(TAG, "System initialization complete");
}

void parse_coordinates(const char *input, double *lat, double *lon) {
    sscanf(input, "%lf,%lf", lat, lon);
}

int ground_setup(void) {
    int BUF_SIZE = 1024;
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    ESP_LOGI("GROUND_SETUP", "Starting ground setup...");
    // initial measurements
    measure_data(&sensor_data);

    // flight zone set up
    ESP_LOGI("GROUND_SETUP", "Do you want to set up flight and landing zones? (y/n)");
    memset(data, 0, BUF_SIZE);
    int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);

    if (len > 0 && (data[0] == 'y' || data[0] == 'Y')) {
        ESP_LOGI("GROUND_SETUP", "Enter the points in the format LAT,LON (e.g., -33.9249,18.4241):");
        coordinate_t box[4];
        for (int i = 0; i < 4; i++) {      
            ESP_LOGI("GROUND_SETUP", "Point %d:", i + 1);
            memset(data, 0, BUF_SIZE);
            len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20000 / portTICK_PERIOD_MS);
            if (len > 0) {
                data[len] = '\0'; // Null-terminate
                ESP_LOGI("GROUND_SETUP", "Received: %s", (char *)data);
                parse_coordinates((char *)data, &box[i].lat, &box[i].lon);                      
            }
        }
        if (store_box_flight_zone(box, 4) == 0) {
            ESP_LOGI(FLIGHT_TAG, "Example box flight zone configured successfully");
        }else {
            return -1
        }  

        ESP_LOGI("GROUND_SETUP", "Now enter the center coordinates of the predicted landing zone:");
        ESP_LOGI("GROUND_SETUP", "Center point:" );
        memset(data, 0, BUF_SIZE);
        current_landing_zone.radius_km = 1; // Default radius
        current_landing_zone.height_m = 100; // Default height
        if (len > 0) {
            data[len] = '\0'; // Null-terminate
            ESP_LOGI("GROUND_SETUP", "Received: %s", (char *)data);
            parse_coordinates((char *)data, &current_landing_zone.center.latitude, 
                &current_landing_zone.center.longitude);
        }

        ESP_LOGI("GROUND_SETUP", "Enter the landing radius (km):" );
        memset(data, 0, BUF_SIZE);
        len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20000 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate
            ESP_LOGI("GROUND_SETUP", "Received: %s", (char *)data);
            current_landing_zone.radius_km = atoi((char *)data);
        }

        ESP_LOGI("GROUND_SETUP", "Enter the estimated ground altitude (km):" );
        memset(data, 0, BUF_SIZE);
        len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20000 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate
            ESP_LOGI("GROUND_SETUP", "Received: %s", (char *)data);
            current_landing_zone.height_m = atoi((char *)data);
        }

        if (store_landing_zone(current_landing_zone) == 0) {
            ESP_LOGI(FLIGHT_TAG, "predicted landing area zone configured successfully");
        }else {
            return -1
        } 

    } else {
        ESP_LOGI("GROUND_SETUP", "Skipping zone setup...");
        return;
    }


    //check motors
    ESP_LOGI("GROUND_SETUP", "Entering motor checks:" );

    ESP_LOGI("GROUND_SETUP", "Balloon release:" );
    balloon_release();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI("GROUND_SETUP", "Balloon lock:" );
    balloon_lock();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI("GROUND_SETUP", "Parachute deploy:" );
    parachute_deploy();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI("GROUND_SETUP", "Parachute lock:" );
    parachute_lock();

    // user control for motors
    while (1) {
        ESP_LOGI("GROUND_SETUP", "Enter command (b=balloon release, l=balloon lock, p=parachute deploy, k=parachute lock, q=quit):" );
        memset(data, 0, BUF_SIZE);
        len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20000 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate
            ESP_LOGI("GROUND_SETUP", "Received: %s", (char *)data);
            if (data[0] == 'b') {
                ESP_LOGI("GROUND_SETUP", "Balloon release:" );
                balloon_release();
            } else if (data[0] == 'l') {
                ESP_LOGI("GROUND_SETUP", "Balloon lock:" );
                balloon_lock();
            } else if (data[0] == 'p') {
                ESP_LOGI("GROUND_SETUP", "Parachute deploy:" );
                parachute_deploy();
            } else if (data[0] == 'k') {
                ESP_LOGI("GROUND_SETUP", "Parachute lock:" );
                parachute_lock();
            } else if (data[0] == 'q') {
                ESP_LOGI("GROUND_SETUP", "Exiting motor checks..." );
                break;
            } else {
                ESP_LOGI("GROUND_SETUP", "Unknown command" );
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    free(data);

    // confirm flight zone stored data
    if (get_flight_zone(&current_flight_zone) == 0) {
        ESP_LOGI(FLIGHT_TAG, "Current flight zone:");
        for (int i = 0; i < current_flight_zone.num_points; i++) {
            ESP_LOGI(FLIGHT_TAG, "  Point %d: %.6f, %.6f", i + 1, 
                     current_flight_zone.points[i].lat, 
                     current_flight_zone.points[i].lon);
        }
    } else {
        ESP_LOGE(FLIGHT_TAG, "Failed to retrieve flight zone data");
    }

    //confirm ground setup is done, return 0 if yes -1 of no

        ESP_LOGI("GROUND_SETUP", "Are you done? (y/n), select no to restart" );
        memset(data, 0, BUF_SIZE);
        len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20000 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (data[0] == 'y' || data[0] == 'Y') {
                ESP_LOGI("GROUND_SETUP", "Ground setup complete." );
                free(data);
                return 0;
            } else {
                ESP_LOGI("GROUND_SETUP", "Restarting ground setup..." );
                free(data);
                return -1;
            }
        }
}

int flight_mode(){
    // measure information from sensors and stuff, every minute, send to "ground station every 5 minutes (use uart for this part)"
    static uint32_t last_measurement_time = 0;
    static uint32_t last_transmission_time = 0;
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Measure sensors every minute (30000 ms)
    if (current_time - last_measurement_time >= 30000) {
        last_measurement_time = current_time;
        ESP_LOGI(TAG, "=== Taking sensor measurements ===");
        
        // Take comprehensive sensor measurements
        measure_data(&sensor_data);
        //check end conditions
        if (end_zone_check()) { // Takes about 6 seconds
            return 0; // End flight mode
        } 
    }
    
    // Transmit to ground station every 5 minutes (300000 ms)
    if (current_time - last_transmission_time >= 300000) {
        transmit_data();
  
        last_transmission_time = current_time;
    }
    
    // Small delay to prevent excessive CPU usage
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    return 1; // Continue flight mode
}

int end_zone_check(){
    // you need to check if you have either crossed the flight zone, or if the balloon has popped (need to check if altitude is decreasing rapidly)
    
    static float previous_altitude = 0.0;
    static float altitude_samples[5] = {0}; // Store last 5 altitude readings
    static int sample_index = 0;
    static bool altitude_tracking_initialized = false;
    static int violation_counts = 0;
    static int previous_time_check = 0;
    int total_checks = 0;

      // Return codes:
    // 0 = Continue flight (no end condition detected)
    // 1 = FLIGHT zone crossed/ burst - Detach
    // -1 = Error in measurements
    
    // Take fresh measurements for this check
    while ((violation_counts > -3) && (total_checks < 15))
    {
        total_checks++;
        int current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        measure_data(&sensor_data, 0);
        // Check for valid GPS and altitude data
        if (!sensor_data.gps_valid || sensor_data.altitude_m <= 0) {
            ESP_LOGW(TAG, "Invalid sensor data for end zone check");
            return -1;
        }

        // === FLIGHT ZONE CROSS CHECK ===
        if (sensor_data.gps_valid) {
            int flight_zone_status = is_within_flight_zone(sensor_data.latitude, sensor_data.longitude);

            if (flight_zone_status == 0) {
                violation_counts++;
                ESP_LOGW(TAG, "Flight zone violation count: %d", violation_counts);
                ESP_LOGW(TAG, "FLIGHT ZONE CROSSED! Detaching balloon");
                ESP_LOGW(TAG, "Current position: %.6f, %.6f", sensor_data.latitude, sensor_data.longitude);
                return 1; // Flight zone crossed
            } else if (flight_zone_status == 1) {
                ESP_LOGW(TAG, "Balloon within flight zone");
                violation_counts--;
            } else if (flight_zone_status == -1) {
                ESP_LOGW(TAG, "No flight zone configured - cannot check boundary");
            }
        }
        // === BALLOON BURST DETECTION ===
        // Initialize altitude tracking on first run
        if (!altitude_tracking_initialized) {
            for (int i = 0; i < 5; i++) {
                altitude_samples[i] = sensor_data.altitude_m;
            }
            previous_altitude = sensor_data.altitude_m;
            altitude_tracking_initialized = true;
        }
        
        // Update altitude samples (rolling buffer)
        altitude_samples[sample_index] = sensor_data.altitude_m;
        sample_index = (sample_index + 1) % 5;
        
        // Calculate altitude change rate (descent rate)
        float altitude_change = sensor_data.altitude_m - previous_altitude;
        float time_diff = (current_time - previous_time_check) / 1000.0; // in seconds
        if (time_diff <= 0) {
            time_diff = 1; // Prevent division by zero
        }
        float descent_rate = -altitude_change / time_diff;

        // Calculate average altitude over last 5 samples for trend analysis
        float altitude_sum = 0;
        for (int i = 0; i < 5; i++) {
            altitude_sum += altitude_samples[i];
        }
        float average_altitude = altitude_sum / 5.0;
        
        // Balloon burst detection criteria:
        // 1. Rapid descent rate (> 5 m/s sustained)
        // 2. Significant altitude drop from recent average (> 100m drop)
        // 3. Consistent downward trend
        
        bool rapid_descent = (descent_rate > 5.0); // Descending faster than 5 m/s
        bool significant_drop = (average_altitude - sensor_data.altitude_m > 100.0);
        bool sustained_descent = true;
        
        // Check if last 5 samples show consistent descent
        for (int i = 1; i < 5; i++) {
            int current_idx = (sample_index - i + 5) % 5;
            int previous_idx = (sample_index - i - 1 + 5) % 5;
            if (altitude_samples[current_idx] >= altitude_samples[previous_idx]) {
                sustained_descent = false;
                break;
            }
        }
        
        ESP_LOGI(TAG, "Altitude: %.1f m, Descent rate: %.2f m/s, Avg: %.1f m", 
                    sensor_data.altitude_m, descent_rate, average_altitude);
        
        // Detect balloon burst
        if (rapid_descent && (significant_drop || sustained_descent)) {
            violation_counts += 5;
        }

        // Balloon burst happened
        if (violation_counts > 5) {
            ESP_LOGW(TAG, "BALLOON BURST DETECTED!");
            ESP_LOGW(TAG, "  Current altitude: %.1f m", sensor_data.altitude_m);
            ESP_LOGW(TAG, "  Descent rate: %.2f m/s", descent_rate);
            ESP_LOGW(TAG, "  Average altitude: %.1f m", average_altitude);
            ESP_LOGW(TAG, "  Rapid descent: %s", rapid_descent ? "YES" : "NO");
            ESP_LOGW(TAG, "  Significant drop: %s", significant_drop ? "YES" : "NO");
            ESP_LOGW(TAG, "  Sustained descent: %s", sustained_descent ? "YES" : "NO");
        return 1;
        }

        previous_time_check = current_time;
        previous_altitude = sensor_data.altitude_m;
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay before next measurement
    }
    

    
   
    
    return 0; // Continue normal flight
}

int falling_mode(){
    // measure information from sensors and stuff, every minute, send to "ground station every 5 minutes (use uart for this part)"
    // Measure sensors every minute (5000 ms)
    static uint32_t last_measurement_time = 0;
    static uint32_t last_transmission_time = 0;
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (current_time - last_measurement_time >= 5000) { // every 5 seconds
        last_measurement_time = current_time;
        ESP_LOGI(TAG, "=== Taking sensor measurements ===");
        // Take comprehensive sensor measurements
        measure_data(&sensor_data);
        if (check_parachute_deploy()) { // takes about a second
            return 0; // End falling mode to enter floating mode
        }
        ;
    }

    if (current_time - last_transmission_time >= 10000) { // every 10 seconds
        transmit_data();
        last_transmission_time = current_time;
    }

    return 1; // Continue falling mode

}

int check_parachute_deploy(){
    // function. that checks if it should deploy parachute based on altitude
    
    static bool parachute_already_deployed = false;
    static float max_altitude_reached = 0.0;
    const float MIN_DEPLOY_ALTITUDE = 500.0; // 500m - minimum safe deployment altitude

    // Return codes:
    // 0 = Do not deploy parachute (continue current flight phase)
    // 1 = Deploy parachute immediately (altitude-based trigger)
    // -1 = Error in measurements
    
    // Check for valid altitude data
    if (sensor_data.altitude_m <= 0) {
        ESP_LOGW(TAG, "Invalid altitude data for parachute deploy check");
        return -1;
    }
    
    // If parachute is already deployed, no need to check again
    if (parachute_already_deployed || sensor_data.parachute_deployed) {
        return 0;
    }

    
    // === ALTITUDE-BASED DEPLOYMENT CRITERIA ===
      
    // Criterion 1: Low altitude safety deployment 
    if (((sensor_data.altitude_m - current_landing_zone.height_m) <= MIN_DEPLOY_ALTITUDE) && sensor_data.max_altitude > 5000.0) { //ensure valid max height reached
        int height_check_count = 1; //confirm how many times it has been below the threshold

        while (height_check_count > -1) {
            vTaskDelay(200 / portTICK_PERIOD_MS); //wait 200 ms
            measure_data(&sensor_data, 0); //take new measurement
            if ((sensor_data.altitude_m - current_landing_zone.height_m) <= MIN_DEPLOY_ALTITUDE) {
                height_check_count++;
            } else {
                height_check_count--;
            }

            if (height_check_count >= 5) { //if it has been below the threshold 5 times in a row, confirm deployment
                parachute_already_deployed = true;
                ESP_LOGW(TAG, "LOW ALTITUDE SAFETY DEPLOYMENT! Deploying parachute");
                ESP_LOGW(TAG, "Current altitude: %.1f m (safety limit: %.1f m)", 
                sensor_data.altitude_m, MIN_DEPLOY_ALTITUDE);
                return 1;
            }
        }
        
    }

    return 0; // Continue normal flight - no deployment needed
}

int floating_mode(){
    // Measure sensors every minute (60000 ms)
    static uint32_t last_measurement_time = 0;
    static uint32_t last_transmission_time = 0;
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (current_time - last_measurement_time >= 5000) { // every 5 seconds
        last_measurement_time = current_time;
        ESP_LOGI(TAG, "=== Taking sensor measurements ===");
        // Take comprehensive sensor measurements
        measure_data(&sensor_data);
        if (check_landed()) { // takes about a second
            return 0; // End floating mode to enter landed mode
        }
    }

    if (current_time - last_transmission_time >= 10000) { // every 10 seconds
        transmit_data();
        last_transmission_time = current_time;
    }

   return 1; // Continue floating mode
}

int check_landed(){
    //check if balloon has landed based on if altitude is stable for a certain amount of time
    
    static float altitude_samples[10] = {0}; // Store last 10 altitude readings for stability analysis
    static int sample_index = 0;
    static bool stability_tracking_initialized = false;
    static uint32_t last_check_time = 0;
    static bool landing_detected = false;
    
    // Take current measurements
    measure_data(&sensor_data);
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Return codes:
    // 0 = Not landed (continue descent/floating)
    // 1 = Landing confirmed (stable altitude detected)
    // -1 = Error in measurements
    
    // Check for valid altitude data
    if (sensor_data.altitude_m <= 0) {
        ESP_LOGW(TAG, "Invalid altitude data for landing check");
        return -1;
    }
    
    // If already confirmed landed, return positive
    if (landing_detected) {
        return 1;
    }
    
    // Initialize stability tracking on first run
    if (!stability_tracking_initialized) {
        for (int i = 0; i < 10; i++) {
            altitude_samples[i] = sensor_data.altitude_m;
        }
        last_check_time = current_time;
        stability_tracking_initialized = true;
        ESP_LOGI(TAG, "Landing detection initialized at %.1f m", sensor_data.altitude_m);
        return 0;
    }
    
    // Update altitude samples (rolling buffer)
    altitude_samples[sample_index] = sensor_data.altitude_m;
    sample_index = (sample_index + 1) % 10;
    
    // === LANDING DETECTION CRITERIA ===
    
    // Calculate altitude statistics over last 10 samples
    float altitude_sum = 0;
    float min_altitude = altitude_samples[0];
    float max_altitude = altitude_samples[0];
    
    for (int i = 0; i < 10; i++) {
        altitude_sum += altitude_samples[i];
        if (altitude_samples[i] < min_altitude) min_altitude = altitude_samples[i];
        if (altitude_samples[i] > max_altitude) max_altitude = altitude_samples[i];
    }
    
    float average_altitude = altitude_sum / 10.0;
    float altitude_variation = max_altitude - min_altitude;
    
    // Criterion 1: Low altitude threshold
    //IS THIS NECESSARY?
    const float GROUND_LEVEL_THRESHOLD = current_landing_zone.height_m + 20; // predicted landing zone height
    bool near_ground = (average_altitude <= GROUND_LEVEL_THRESHOLD);
    
    // Criterion 2: Altitude stability (low variation)
    const float STABILITY_THRESHOLD = 5.0; // 5m variation over 10 samples
    bool altitude_stable = (altitude_variation <= STABILITY_THRESHOLD);
    

    
    // Criterion 3: IMU indicates stationary on ground (if available)
    bool no_movement = false;
    if (sensor_data.accel_x != 0 || sensor_data.accel_y != 0 || sensor_data.accel_z != 0) {
        // Calculate total acceleration magnitude
        float accel_magnitude = sqrt(sensor_data.accel_x * sensor_data.accel_x + 
                                   sensor_data.accel_y * sensor_data.accel_y + 
                                   sensor_data.accel_z * sensor_data.accel_z);
        
        // On ground, expect ~1g (9.81 m/s²) with small tolerance
        const float GRAVITY = 1.0; // 1g
        const float GROUND_TOLERANCE = 0.2; // ±0.2g tolerance for ground contact
        no_movement = (fabs(accel_magnitude - GRAVITY) <= GROUND_TOLERANCE);
        
        ESP_LOGI(TAG, "Accelerometer magnitude: %.3f g (ground detection: %s)", 
                 accel_magnitude, no_movement ? "YES" : "NO");
    }
    
    // === STABILITY TIME TRACKING ===
    
    // Check if current conditions indicate stability    
    if (altitude_stable && no_movement){
        return 1;
    }

    // === EMERGENCY LANDING DETECTION ===
    
    // Very low altitude with any stability indicates emergency landing
    const float EMERGENCY_ALTITUDE = 50.0; // 50m - definitely landed
    if (average_altitude <= EMERGENCY_ALTITUDE && altitude_stable) {
        ESP_LOGW(TAG, "EMERGENCY LANDING DETECTED at very low altitude!");
        ESP_LOGW(TAG, "  Altitude: %.1f m (emergency threshold: %.1f m)", 
                 average_altitude, EMERGENCY_ALTITUDE);
        landing_detected = true;
        return 1;
    }
    

    
    return 0; // Not yet landed
}

void land_mode(){
        // Measure sensors every minute (60000 ms)
    static uint32_t last_measurement_time = 0;
    static uint32_t last_transmission_time = 0;
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (current_time - last_measurement_time >= 50000) { // every 50 seconds
        ESP_LOGI(TAG, "=== Taking sensor measurements ===");
        
        // Take comprehensive sensor measurements
        measure_data(&sensor_data);
        last_measurement_time = current_time;
    }

    if (current_time - last_transmission_time >= 300000) { // every 5 minutes
        transmit_data();
        last_transmission_time = current_time;
    }

}

void main_loop(){

    while (1){
        switch (current_state) {
            case STATE_INITIALIZATION:
                ESP_LOGI(TAG, "State: INITIALIZATION");
                if (system_init() != 0) {
                    ESP_LOGE(TAG, "System initialization failed, retrying...");
                    vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait before retrying
                    continue; // Retry initialization
                }else {
                    ESP_LOGI(TAG, "System initialization successful");
                    current_state = STATE_SETUP;
                }
                break;
                
            case STATE_SETUP:
                ESP_LOGI(TAG, "State: SETUP");
                if (ground_setup() != 0) {
                    ESP_LOGE(TAG, "Ground setup failed or incomplete, restarting...");
                    current_state = STATE_INITIALIZATION; // Restart setup
                    vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait before retrying
                    continue;
                } else {
                    ESP_LOGI(TAG, "Ground setup complete");
                }
                current_state = STATE_ASCENT;
                break;
                
            case STATE_ASCENT:
                ESP_LOGI(TAG, "State: ASCENT");
                if (flight_mode()) {
                    current_state = STATE_ASCENT; // Continue ascent
                } else {
                    current_state = STATE_DESCENT; // Transition to descent
                    balloon_release();
                }
                break;

            case STATE_DESCENT:
                ESP_LOGI(TAG, "State: DESCENT");
                if (falling_mode()) {
                    current_state = STATE_DESCENT; // Continue descent
                } else {
                    current_state = STATE_FLOATING; // Transition to floating
                    parachute_deploy();
                }

                break;
                
            case STATE_FLOATING:
                ESP_LOGI(TAG, "State: FLOATING");

                if (floating_mode()) {
                    current_state = STATE_FLOATING; // Continue floating
                } else {
                    current_state = STATE_LANDED; // Transition to landed
                    
                }


            case STATE_LANDED:
                ESP_LOGI(TAG, "State: LANDED");
                powersave();
                
                // Optionally, could implement recovery procedures here
                break;
                
            default:
                ESP_LOGE(TAG, "Unknown state!");
                current_state = STATE_INITIALIZATION; // Reset to safe state
                break;
        }
        
        // Small delay to prevent excessive CPU usage
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}