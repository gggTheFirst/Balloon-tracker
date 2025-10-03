#include "imu.h"

// Tag for ESP_LOG output (helps filter logs)
static const char *TAG = "IMU_TEST";


int imu_init() {
    i2c_master_init();
    
    // Initialize the MPU9250
    if (mpu9250_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU9250");
        return -1;
    }
    
    // Initialize the BMP280
    if (bmp280_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
        return -1;
    }
    return 0;
}

esp_err_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr, &reg_addr, 1, data, 1, 1000 / portTICK_PERIOD_MS);
}


int i2c_master_init() {
    // Create a config struct and fill in the settings
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = I2C_MASTER_SDA_IO, 
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, 
        .scl_pullup_en = GPIO_PULLUP_ENABLE, 
        .master.clk_speed = I2C_MASTER_FREQ_HZ, 
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)  {
        ESP_LOGE(TAG, "I2C init failed");
        return -1; 
    }
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    esp_err_t ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C installation failed");
        return -1;
    }

    // Check if sensors are connected by reading their ID registers
    uint8_t mpu_id = 0, bmp_id = 0;
    // Read WHO_AM_I register from MPU9250 
    ret = i2c_read_byte(MPU9250_ADDR, 0x75, &mpu_id); // 0x75 is WHO_AM_I
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU9250 WHO_AM_I: 0x%02X", mpu_id);
    } else {
        ESP_LOGE(TAG, "MPU9250 not detected");
    }
    // Read ID register from BMP280 
    ret = i2c_read_byte(BMP280_ADDR, 0xD0, &bmp_id); // 0xD0 is ID
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BMP280 ID: 0x%02X", bmp_id);
    } else {
        ESP_LOGE(TAG, "BMP280 not detected");
    }

    
    return 0;
}



// Function to read multiple bytes from I2C device
esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// Function to combine high and low bytes into 16-bit value
int16_t combine_bytes(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

// Initialize MPU9250 for full operation
esp_err_t mpu9250_init() {
    esp_err_t ret;
    
    // Wake up the device (clear sleep bit)
    uint8_t wake_cmd = 0x00;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_ADDR, (uint8_t[]){0x6B, wake_cmd}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU9250");
        return ret;
    }
    
    // Set accelerometer range to ±2g
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_ADDR, (uint8_t[]){0x1C, 0x00}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }
    
    // Set gyroscope range to ±250°/s
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_ADDR, (uint8_t[]){0x1B, 0x00}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU9250 initialized successfully");
    return ESP_OK;
}

// Read all data from MPU9250
esp_err_t read_mpu9250_data(mpu9250_data_t *data) {
    uint8_t raw_data[14];
    esp_err_t ret;
    
    // Read accelerometer, temperature, and gyroscope data (14 bytes starting from 0x3B)
    ret = i2c_read_bytes(MPU9250_ADDR, 0x3B, raw_data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU9250 data");
        return ret;
    }
    
    // Parse accelerometer data (registers 0x3B-0x40)
    data->accel_x = accel_to_g(combine_bytes(raw_data[0], raw_data[1]));
    data->accel_y = accel_to_g(combine_bytes(raw_data[2], raw_data[3]));
    data->accel_z = accel_to_g(combine_bytes(raw_data[4], raw_data[5]));

    // Parse temperature data (registers 0x41-0x42)
    data->temp = temp_to_celsius(combine_bytes(raw_data[6], raw_data[7]));
    
    // Parse gyroscope data (registers 0x43-0x48)
    data->gyro_x = gyro_to_dps(combine_bytes(raw_data[8], raw_data[9]));
    data->gyro_y = gyro_to_dps(combine_bytes(raw_data[10], raw_data[11]));
    data->gyro_z = gyro_to_dps(combine_bytes(raw_data[12], raw_data[13]));

    return ESP_OK;
}

// Global calibration data
bmp280_calib_t bmp280_calib;
int32_t t_fine = 0; // Global variable for pressure calculation

// Initialize BMP280 and read calibration data
esp_err_t bmp280_init() {
    esp_err_t ret;
    
    // Set BMP280 to normal mode, temperature oversampling x2, pressure oversampling x16
    uint8_t ctrl_meas = (0x02 << 5) | (0x05 << 2) | 0x03; // osrs_t=2, osrs_p=16, mode=normal
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_ADDR, (uint8_t[]){0xF4, ctrl_meas}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BMP280");
        return ret;
    }
    
    // Set config register (standby time = 1000ms, filter = 4, SPI 3-wire disabled)
    uint8_t config = (0x05 << 5) | (0x02 << 2) | 0x00;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_ADDR, (uint8_t[]){0xF5, config}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BMP280 config");
        return ret;
    }
    
    // Read calibration data (24 bytes starting from 0x88)
    uint8_t calib_data[24];
    ret = i2c_read_bytes(BMP280_ADDR, 0x88, calib_data, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 calibration data");
        return ret;
    }
    
    // Parse calibration coefficients
    bmp280_calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    bmp280_calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    bmp280_calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    bmp280_calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    bmp280_calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    bmp280_calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    bmp280_calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    bmp280_calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    bmp280_calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    bmp280_calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    bmp280_calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    bmp280_calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];
    
    ESP_LOGI(TAG, "BMP280 initialized with calibration data");
    ESP_LOGI(TAG, "T1=%u, T2=%d, T3=%d", bmp280_calib.dig_T1, bmp280_calib.dig_T2, bmp280_calib.dig_T3);
    
    return ESP_OK;
}

// Convert raw temperature using BMP280 calibration (from datasheet)
float bmp280_compensate_temperature(int32_t adc_T) {
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) * ((int32_t)bmp280_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    return T / 100.0; // Convert to Celsius
}

// Convert raw pressure using BMP280 calibration (from datasheet)
float bmp280_compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_calib.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // Avoid exception caused by division by zero
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
    
    return p / 256.0; // Convert to hPa
}

// Read BMP280 data
esp_err_t read_bmp280_data(bmp280_data_t *data) {
    uint8_t raw_data[6];
    esp_err_t ret;
    
    // Read pressure and temperature data (6 bytes starting from 0xF7)
    ret = i2c_read_bytes(BMP280_ADDR, 0xF7, raw_data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 data");
        return ret;
    }
    
    data->pressure_hpa = bmp280_compensate_pressure((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));
    data->temperature_c = bmp280_compensate_temperature((raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4));
    data->altitude_m = calculate_altitude(data->pressure_hpa);
    
    return ESP_OK;
}

float calculate_altitude(float pressure_hpa) {
    // Standard atmospheric pressure at sea level: 1013.25 hPa
    const float SEA_LEVEL_PRESSURE_HPA = 1013.25;
    return 44330.0 * (1.0 - pow(pressure_hpa / SEA_LEVEL_PRESSURE_HPA, 0.1903));
}

// Convert raw accelerometer data to g-force
float accel_to_g(int16_t raw_accel) {
    return raw_accel / 16384.0;  // For ±2g range
}

// Convert raw gyroscope data to degrees per second
float gyro_to_dps(int16_t raw_gyro) {
    return raw_gyro / 131.0;  // For ±250°/s range
}

// Convert raw temperature data to Celsius
// NOTE: MPU9250 temperature sensor is known to be inaccurate and can be off by several degrees
// Use BMP280 temperature for accurate readings
float temp_to_celsius(int16_t raw_temp) {
    // Corrected MPU9250 temperature formula from datasheet
    return (raw_temp / 333.87) + 21.0;  // More accurate formula
}

// Alternative temperature calculation method
float temp_to_celsius_alt(int16_t raw_temp) {
    // This is the official formula from MPU9250 datasheet
    return ((float)raw_temp - 21.0) / 333.87 + 21.0;
}

void app_main() {

imu_init();

    // --- Main loop to read all sensor data ---
    while (1) {
        mpu9250_data_t mpu_data;
        bmp280_data_t bmp_data;
        
        // Read all MPU9250 data
        if (read_mpu9250_data(&mpu_data) == ESP_OK) {
            ESP_LOGI(TAG, "=== MPU9250 Data ===");
            ESP_LOGI(TAG, "Accelerometer: X=%d, Y=%d, Z=%d", 
                     mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z);

            ESP_LOGI(TAG, "Gyroscope(°/s): X=%d, Y=%d, Z=%d", 
                     mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z);
            ESP_LOGI(TAG, "Gyroscope (°/s): X=%.3f, Y=%.3f, Z=%.3f", 
                     gyro_to_dps(mpu_data.gyro_x), gyro_to_dps(mpu_data.gyro_y), gyro_to_dps(mpu_data.gyro_z));
            ESP_LOGI(TAG, "MPU9250 Temp Alt: %.2f°C [Alternative calculation]", 
                     temp_to_celsius_alt(mpu_data.temp));
        }
        
        // Read all BMP280 data
        if (read_bmp280_data(&bmp_data) == ESP_OK) {
            ESP_LOGI(TAG, "=== BMP280 Data ===");
            ESP_LOGI(TAG, "Pressure: %.2f hPa", bmp_data.pressure_hpa);
            ESP_LOGI(TAG, "Temperature: %.2f°C", bmp_data.temperature_c);
        }
        
        ESP_LOGI(TAG, "========================");

        // Wait 1 second before next reading
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}