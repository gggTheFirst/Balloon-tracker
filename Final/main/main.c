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

static const char *TAG = "BALLOON_TRACKER";

// Balloon states
typedef enum {
    STATE_INITIALIZATION,
    STATE_SETUP,
    STATE_ASCENT,
    STATE_FLIGHT,
    STATE_DESCENT,
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
    
} measurement_data_t;

// Global variables
static measurement_data_t sensor_data;
static balloon_state_t current_state = STATE_INITIALIZATION;
static double ground_altitude = 0.0;  // Reference ground level
static bool altitude_calibrated = false;

// Command interface configuration
#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
static QueueHandle_t uart_queue;

// Manual control flags
static bool manual_mode = false;
static bool force_balloon_release = false;
static bool force_parachute_deploy = false;
static bool simulate_flight_zone_violation = false;

// Convert NMEA coordinate string to decimal degrees
double nmea_string_to_decimal(const char* nmea_coord) {
    if (strlen(nmea_coord) < 5) return 0.0;
    
    // Extract direction (last character)
    char direction = nmea_coord[strlen(nmea_coord) - 1];
    
    // Create a copy without the direction
    char coord_str[20];
    strncpy(coord_str, nmea_coord, strlen(nmea_coord) - 2); // Remove ",N" or ",S" etc.
    coord_str[strlen(nmea_coord) - 2] = '\0';
    
    double coord_value = atof(coord_str);
    
    // Convert DDMM.MMMMM to decimal degrees
    int degrees = (int)(coord_value / 100);
    double minutes = coord_value - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    
    // Apply direction (negative for South/West)
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

// Initialize UART for command interface
void init_command_interface(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    ESP_LOGI(TAG, "📡 Command interface initialized on UART0 (115200 baud)");
    ESP_LOGI(TAG, "Available commands:");
    ESP_LOGI(TAG, "  RELEASE - Force balloon release");
    ESP_LOGI(TAG, "  PARACHUTE - Deploy parachute");  
    ESP_LOGI(TAG, "  LOCK - Lock both servos");
    ESP_LOGI(TAG, "  MANUAL - Toggle manual mode");
    ESP_LOGI(TAG, "  SIMULATE_VIOLATION - Simulate flight zone violation");
    ESP_LOGI(TAG, "  STATUS - Show current status");
    ESP_LOGI(TAG, "  HELP - Show this help");
}

// Process incoming commands
void process_command(const char* command) {
    char cmd_upper[32];
    strncpy(cmd_upper, command, sizeof(cmd_upper) - 1);
    cmd_upper[sizeof(cmd_upper) - 1] = '\0';
    
    // Convert to uppercase
    for (int i = 0; cmd_upper[i]; i++) {
        if (cmd_upper[i] >= 'a' && cmd_upper[i] <= 'z') {
            cmd_upper[i] = cmd_upper[i] - 'a' + 'A';
        }
    }

    ESP_LOGI(TAG, "🔧 Command received: %s", cmd_upper);

    if (strncmp(cmd_upper, "RELEASE", 7) == 0) {
        ESP_LOGW(TAG, "🚨 MANUAL BALLOON RELEASE TRIGGERED!");
        balloon_release();
        sensor_data.balloon_released = true;
        force_balloon_release = true;
        
    } else if (strncmp(cmd_upper, "PARACHUTE", 9) == 0) {
        ESP_LOGW(TAG, "🪂 MANUAL PARACHUTE DEPLOYMENT TRIGGERED!");
        parachute_deploy();
        sensor_data.parachute_deployed = true;
        force_parachute_deploy = true;
        
    } else if (strncmp(cmd_upper, "LOCK", 4) == 0) {
        ESP_LOGI(TAG, "🔒 LOCKING ALL SERVOS");
        balloon_lock();
        parachute_lock();
        sensor_data.balloon_released = false;
        sensor_data.parachute_deployed = false;
        force_balloon_release = false;
        force_parachute_deploy = false;
        
    } else if (strncmp(cmd_upper, "MANUAL", 6) == 0) {
        manual_mode = !manual_mode;
        ESP_LOGI(TAG, "🎛️  Manual mode: %s", manual_mode ? "ENABLED" : "DISABLED");
        
    } else if (strncmp(cmd_upper, "SIMULATE_VIOLATION", 18) == 0) {
        simulate_flight_zone_violation = !simulate_flight_zone_violation;
        ESP_LOGW(TAG, "⚠️  Flight zone violation simulation: %s", 
                 simulate_flight_zone_violation ? "ACTIVE" : "DISABLED");
                 
    } else if (strncmp(cmd_upper, "STATUS", 6) == 0) {
        ESP_LOGI(TAG, "📊 CURRENT STATUS:");
        ESP_LOGI(TAG, "  State: %d", current_state);
        ESP_LOGI(TAG, "  Manual Mode: %s", manual_mode ? "ON" : "OFF");
        ESP_LOGI(TAG, "  Balloon Released: %s", sensor_data.balloon_released ? "YES" : "NO");
        ESP_LOGI(TAG, "  Parachute Deployed: %s", sensor_data.parachute_deployed ? "YES" : "NO");
        ESP_LOGI(TAG, "  Altitude: %.1f m", sensor_data.altitude_m - ground_altitude);
        ESP_LOGI(TAG, "  GPS Valid: %s", sensor_data.gps_valid ? "YES" : "NO");
        if (sensor_data.gps_valid) {
            ESP_LOGI(TAG, "  Position: %.6f, %.6f", sensor_data.latitude, sensor_data.longitude);
        }
        
    } else if (strncmp(cmd_upper, "HELP", 4) == 0) {
        ESP_LOGI(TAG, "🆘 AVAILABLE COMMANDS:");
        ESP_LOGI(TAG, "  RELEASE - Force balloon release");
        ESP_LOGI(TAG, "  PARACHUTE - Deploy parachute");  
        ESP_LOGI(TAG, "  LOCK - Lock both servos");
        ESP_LOGI(TAG, "  MANUAL - Toggle manual mode");
        ESP_LOGI(TAG, "  SIMULATE_VIOLATION - Simulate flight zone violation");
        ESP_LOGI(TAG, "  STATUS - Show current status");
        ESP_LOGI(TAG, "  HELP - Show this help");
        
    } else {
        ESP_LOGW(TAG, "❓ Unknown command: %s (type HELP for available commands)", cmd_upper);
    }
}

// Command monitoring task
void command_task(void *pvParameters) {
    char data[BUF_SIZE];
    int len;
    
    ESP_LOGI(TAG, "🎮 Command monitoring task started");
    
    while (1) {
        len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            data[len] = '\0';
            
            // Remove newlines and carriage returns
            for (int i = 0; i < len; i++) {
                if (data[i] == '\n' || data[i] == '\r') {
                    data[i] = '\0';
                    break;
                }
            }
            
            if (strlen(data) > 0) {
                process_command(data);
            }
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// Estimate altitude from pressure (simple approximation)
double pressure_to_altitude(double pressure_hpa) {
    const double sea_level_pressure = 1013.25; // Standard sea level pressure
    const double temperature_lapse = 0.0065;   // Temperature lapse rate (K/m)
    const double temperature_sea_level = 288.15; // Sea level temperature (K)
    const double gravity = 9.80665;            // Gravitational acceleration
    const double gas_constant = 287.053;       // Specific gas constant for air
    
    // Barometric formula
    double altitude = (temperature_sea_level / temperature_lapse) * 
                     (1.0 - pow(pressure_hpa / sea_level_pressure, 
                               (gas_constant * temperature_lapse) / gravity));
    
    return altitude;
}

// Setup sequence - configure flight and landing zones
void setup_flight_zones(void) {
    ESP_LOGI(TAG, "=== BALLOON TRACKER SETUP SEQUENCE ===");
    
    // Initialize flight zone system
    if (flight_zone_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize flight zone system");
        return;
    }
    
    // Setup default flight zone (circular, 20km radius around Cape Town)
    double flight_center_lat = -33.9249;  // Cape Town coordinates
    double flight_center_lon = 18.4241;
    double flight_radius = 20.0; // 20 km radius
    
    if (store_circular_flight_zone(flight_center_lat, flight_center_lon, flight_radius) == 0) {
        ESP_LOGI(TAG, "✓ Flight zone configured: Center(%.4f, %.4f), Radius=%.1f km", 
                 flight_center_lat, flight_center_lon, flight_radius);
    } else {
        ESP_LOGE(TAG, "✗ Failed to configure flight zone");
    }
    
    // Setup landing zone (for future use)
    ESP_LOGI(TAG, "✓ Landing zone: Emergency recovery protocol enabled");
    ESP_LOGI(TAG, "✓ Setup sequence completed successfully");
}

// Read all sensor data
void read_sensor_data(measurement_data_t *data) {
    mpu9250_data_t imu_data;
    bmp280_data_t baro_data;
    char lat_str[20], lon_str[20];
    
    // Read IMU data
    if (read_mpu9250_data(&imu_data) == ESP_OK) {
        data->accel_x = accel_to_g(imu_data.accel_x);
        data->accel_y = accel_to_g(imu_data.accel_y);
        data->accel_z = accel_to_g(imu_data.accel_z);
        data->gyro_x = gyro_to_dps(imu_data.gyro_x);
        data->gyro_y = gyro_to_dps(imu_data.gyro_y);
        data->gyro_z = gyro_to_dps(imu_data.gyro_z);
        data->inside_temperature = temp_to_celsius(imu_data.temp);
    }
    
    // Read barometric data
    if (read_bmp280_data(&baro_data) == ESP_OK) {
        data->outside_temperature = baro_data.temperature_c;
        data->pressure_hpa = baro_data.pressure_hpa;
        data->altitude_m = pressure_to_altitude(baro_data.pressure_hpa);
        
        // Calibrate ground altitude on first valid reading
        if (!altitude_calibrated && data->altitude_m > -1000) {
            ground_altitude = data->altitude_m;
            altitude_calibrated = true;
            ESP_LOGI(TAG, "Ground altitude calibrated: %.1f m", ground_altitude);
        }
    }
    
    // Read GPS data
    if (get_gps_lat_lon(lat_str, sizeof(lat_str), lon_str, sizeof(lon_str)) == 0) {
        data->latitude = nmea_string_to_decimal(lat_str);
        data->longitude = nmea_string_to_decimal(lon_str);
        data->gps_valid = true;
        
        // Check flight zone
        int zone_status = is_within_flight_zone(data->latitude, data->longitude);
        data->within_flight_zone = (zone_status == 1);
        
        // Apply simulation override if active
        if (simulate_flight_zone_violation) {
            data->within_flight_zone = false;
        }
    } else {
        data->gps_valid = false;
        data->within_flight_zone = true; // Assume OK if no GPS
    }
    
    // Apply manual control overrides
    if (force_balloon_release) {
        data->balloon_released = true;
    }
    if (force_parachute_deploy) {
        data->parachute_deployed = true;
    }
    
    // Update state information
    data->state = current_state;
}

// Display sensor data on monitor
void display_sensor_data(const measurement_data_t *data) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                BALLOON SENSOR DATA                   ║");
    if (manual_mode || simulate_flight_zone_violation) {
        ESP_LOGI(TAG, "║ MODE: %s%s                                 ║",
                 manual_mode ? "MANUAL " : "",
                 simulate_flight_zone_violation ? "SIMULATION" : "");
    }
    ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════╣");
    
    // State and status
    const char* state_names[] = {"INIT", "SETUP", "ASCENT", "FLIGHT", "DESCENT", "LANDED"};
    ESP_LOGI(TAG, "║ State: %-10s │ Flight Zone: %s │ Alt: %6.0f m ║", 
             state_names[data->state],
             data->within_flight_zone ? "✓ OK " : "✗ OUT",
             data->altitude_m - ground_altitude);
    
    // Temperature data
    ESP_LOGI(TAG, "║ Temperature: Inside=%.1f°C │ Outside=%.1f°C          ║",
             data->inside_temperature, data->outside_temperature);
    
    // Pressure and altitude
    ESP_LOGI(TAG, "║ Pressure: %.1f hPa │ Ground+%.0fm │ Total: %.0fm      ║",
             data->pressure_hpa, 
             data->altitude_m - ground_altitude,
             data->altitude_m);
    
    // GPS data
    if (data->gps_valid) {
        ESP_LOGI(TAG, "║ GPS: %.6f, %.6f (VALID)                    ║",
                 data->latitude, data->longitude);
    } else {
        ESP_LOGI(TAG, "║ GPS: NO FIX AVAILABLE                                ║");
    }
    
    // IMU data
    ESP_LOGI(TAG, "║ Accel: X=%.2fg Y=%.2fg Z=%.2fg                        ║",
             data->accel_x, data->accel_y, data->accel_z);
    ESP_LOGI(TAG, "║ Gyro:  X=%.1f°/s Y=%.1f°/s Z=%.1f°/s                   ║",
             data->gyro_x, data->gyro_y, data->gyro_z);
    
    // Status indicators
    ESP_LOGI(TAG, "║ Parachute: %s │ Balloon: %s                      ║",
             data->parachute_deployed ? "DEPLOYED" : "STOWED  ",
             data->balloon_released ? "RELEASED" : "ATTACHED");
    
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════╝");
}

// State machine for balloon control
void balloon_state_machine(measurement_data_t *data) {
    static int ascent_counter = 0;
    static int descent_counter = 0;
    double relative_altitude = data->altitude_m - ground_altitude;
    
    // Skip automatic state machine if in manual mode
    if (manual_mode) {
        ESP_LOGD(TAG, "Manual mode active - state machine paused");
        return;
    }
    
    switch (current_state) {
        case STATE_INITIALIZATION:
            if (altitude_calibrated && data->gps_valid) {
                current_state = STATE_SETUP;
                ESP_LOGI(TAG, ">>> Transitioning to SETUP state");
            }
            break;
            
        case STATE_SETUP:
            // Wait for stable readings
            current_state = STATE_ASCENT;
            ESP_LOGI(TAG, ">>> Transitioning to ASCENT state");
            break;
            
        case STATE_ASCENT:
            // Detect ascent (altitude increasing)
            if (relative_altitude > 100) { // Above 100m
                ascent_counter++;
                if (ascent_counter > 3) { // Stable ascent
                    current_state = STATE_FLIGHT;
                    ESP_LOGI(TAG, ">>> Transitioning to FLIGHT state");
                }
            }
            break;
            
        case STATE_FLIGHT:
            // Check flight zone violations
            if (!data->within_flight_zone && data->gps_valid) {
                ESP_LOGW(TAG, "⚠️  FLIGHT ZONE VIOLATION DETECTED!");
                ESP_LOGW(TAG, "🚨 TRIGGERING BALLOON RELEASE!");
                
                balloon_release();
                data->balloon_released = true;
                current_state = STATE_DESCENT;
                ESP_LOGI(TAG, ">>> Emergency transition to DESCENT state");
            }
            // Also check for natural descent
            else if (relative_altitude < (data->altitude_m - 50)) { // Descending
                descent_counter++;
                if (descent_counter > 5) {
                    current_state = STATE_DESCENT;
                    ESP_LOGI(TAG, ">>> Natural transition to DESCENT state");
                }
            }
            break;
            
        case STATE_DESCENT:
            // Deploy parachute at low altitude (200m above ground)
            if (relative_altitude <= 200 && !data->parachute_deployed) {
                ESP_LOGW(TAG, "🪂 DEPLOYING PARACHUTE AT %.0f m!", relative_altitude);
                
                parachute_deploy();
                data->parachute_deployed = true;
            }
            
            // Check for landing
            if (relative_altitude <= 10) { // Within 10m of ground
                current_state = STATE_LANDED;
                ESP_LOGI(TAG, ">>> Transitioning to LANDED state");
            }
            break;
            
        case STATE_LANDED:
            ESP_LOGI(TAG, "🏁 BALLOON HAS LANDED - Mission Complete!");
            break;
    }
}

// Main sensor monitoring task
void sensor_monitoring_task(void *pvParameter) {
    ESP_LOGI(TAG, "🚀 Sensor monitoring task started");
    
    while (1) {
        // Read all sensor data
        read_sensor_data(&sensor_data);
        
        // Run state machine
        balloon_state_machine(&sensor_data);
        
        // Display data
        display_sensor_data(&sensor_data);
        
        // Wait 1 minute before next measurement
        vTaskDelay(60000 / portTICK_PERIOD_MS); // 60 seconds
    }
}

// Initialize all systems
void initialize_systems(void) {
    ESP_LOGI(TAG, "🔧 Initializing balloon tracker systems...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    // Initialize I2C and sensors
    if (i2c_master_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }
    
    if (mpu9250_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU9250");
    }
    
    if (bmp280_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
    }
    
    // Initialize temperature sensor ADC
    configure_adc();
    
    // Initialize servos
    configure_servos();
    
    // Initialize command interface
    init_command_interface();
    
    // Initialize in locked position
    balloon_lock();
    parachute_lock();
    
    ESP_LOGI(TAG, "✓ All systems initialized successfully");
}

void app_main(void)
{
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║              WEATHER BALLOON TRACKER v1.0             ║");
    ESP_LOGI(TAG, "║                    Starting Up...                      ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");
    
    // Initialize all systems
    initialize_systems();
    
    // Wait for systems to stabilize
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Run setup sequence
    setup_flight_zones();
    
    // Start main monitoring task
    xTaskCreate(sensor_monitoring_task, "Sensor Monitor", 8192, NULL, 5, NULL);
    
    // Start command interface task
    xTaskCreate(command_task, "Command Interface", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "🎈 Balloon tracker is now operational!");
    ESP_LOGI(TAG, "📊 Sensor readings every 60 seconds");
    ESP_LOGI(TAG, "🛡️  Flight zone monitoring active");
    ESP_LOGI(TAG, "🪂 Emergency systems armed");
    ESP_LOGI(TAG, "🎮 Command interface ready (type HELP for commands)");
}
