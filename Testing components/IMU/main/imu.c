#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

// --- I2C configuration constants ---
// I2C_NUM_0: Use I2C hardware port 0 on the ESP32
#define I2C_MASTER_NUM              I2C_NUM_0
// SCL (clock) and SDA (data) pin numbers for ESP32
#define I2C_MASTER_SCL_IO           21
#define I2C_MASTER_SDA_IO           19
// I2C clock speed (400kHz is 'fast mode')
#define I2C_MASTER_FREQ_HZ          400000
// I2C addresses for the sensors 
#define MPU9250_ADDR                0x68 // IMU sensor
#define BMP280_ADDR                 0x76 // Barometer sensor

// Tag for ESP_LOG output (helps filter logs)
static const char *TAG = "IMU_TEST";

// --- I2C Master Initialization ---
// This function sets up the ESP32 to act as an I2C master.
// It configures the pins, enables pullups, sets the speed, and installs the driver.
esp_err_t i2c_master_init() {
    // Create a config struct and fill in the settings
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, // ESP32 will control the bus
        .sda_io_num = I2C_MASTER_SDA_IO, // Data line pin
        .scl_io_num = I2C_MASTER_SCL_IO, // Clock line pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Enable pull-up resistor for SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Enable pull-up resistor for SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Set clock speed
    };
    // Apply the config to the I2C hardware
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err; // If config fails, return error
    // Install the I2C driver so ESP32 can use the bus
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// --- I2C Read Helper ---
// This function reads a single byte from a register on a device over I2C.
// dev_addr: I2C address of the device (e.g., MPU9250)
// reg_addr: Register address to read from
// data: Pointer to store the result
// Returns ESP_OK if successful
esp_err_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    // i2c_master_write_read_device does the following:
    // 1. Sends the register address to the device
    // 2. Reads one byte from that register
    // 3. Uses a timeout (here, 1000ms)
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr, &reg_addr, 1, data, 1, 1000 / portTICK_PERIOD_MS);
}

void app_main() {
    // --- Step 1: Initialize I2C hardware ---
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }

    // --- Step 2: Check if sensors are connected by reading their ID registers ---
    uint8_t mpu_id = 0, bmp_id = 0;
    // Read WHO_AM_I register from MPU9250 (should return a known value if present)
    ret = i2c_read_byte(MPU9250_ADDR, 0x75, &mpu_id); // 0x75 is WHO_AM_I
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU9250 WHO_AM_I: 0x%02X", mpu_id);
    } else {
        ESP_LOGE(TAG, "MPU9250 not detected");
    }
    // Read ID register from BMP280 (should return a known value if present)
    ret = i2c_read_byte(BMP280_ADDR, 0xD0, &bmp_id); // 0xD0 is ID
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BMP280 ID: 0x%02X", bmp_id);
    } else {
        ESP_LOGE(TAG, "BMP280 not detected");
    }

    // --- Step 3: Main loop to read sensor data live ---
    while (1) {
        // --- Read accelerometer data from MPU9250 ---
        // Each axis is stored in two registers (high and low byte), but here we just read the high byte for demo
        uint8_t accel_x = 0, accel_y = 0, accel_z = 0;
        i2c_read_byte(MPU9250_ADDR, 0x3B, &accel_x); // ACCEL_XOUT_H
        i2c_read_byte(MPU9250_ADDR, 0x3D, &accel_y); // ACCEL_YOUT_H
        i2c_read_byte(MPU9250_ADDR, 0x3F, &accel_z); // ACCEL_ZOUT_H
        ESP_LOGI(TAG, "Accel X: %d, Y: %d, Z: %d", accel_x, accel_y, accel_z);

        // --- Read barometer data from BMP280 ---
        // Pressure is stored in 3 registers (MSB, LSB, XLSB), here we just read MSB for demo
        uint8_t pressure_msb = 0;
        i2c_read_byte(BMP280_ADDR, 0xF7, &pressure_msb); // Pressure MSB
        ESP_LOGI(TAG, "Barometer Pressure MSB: %d", pressure_msb);

        // --- Wait before next read (100ms) ---
        vTaskDelay(100 / portTICK_PERIOD_MS); // 0.1 second delay
    }
}