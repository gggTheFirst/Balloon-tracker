#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- I2C Configuration Constants ---
#define I2C_MASTER_NUM              I2C_NUM_0    // I2C hardware port 0 on ESP32
#define I2C_MASTER_SCL_IO           21           // SCL (clock) pin for ESP32
#define I2C_MASTER_SDA_IO           19           // SDA (data) pin for ESP32
#define I2C_MASTER_FREQ_HZ          400000       // I2C clock speed (400kHz fast mode)

// --- Sensor I2C Addresses ---
#define MPU9250_ADDR                0x68         // MPU9250 IMU sensor address
#define BMP280_ADDR                 0x76         // BMP280 barometer sensor address

// --- Data Structures ---

/**
 * @brief Structure to hold all MPU9250 sensor data
 */
typedef struct {
    int16_t accel_x, accel_y, accel_z;  // 3-axis accelerometer data (raw)
    int16_t temp;                       // Temperature data (raw)
    int16_t gyro_x, gyro_y, gyro_z;     // 3-axis gyroscope data (raw)
    int16_t mag_x, mag_y, mag_z;        // 3-axis magnetometer data (placeholder)
} mpu9250_data_t;

/**
 * @brief Structure to hold BMP280 calibration coefficients
 */
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_t;

/**
 * @brief Structure to hold BMP280 sensor data
 */
typedef struct {
    float pressure_hpa;         // Calibrated pressure in hPa
    float temperature_c;        // Calibrated temperature in Celsius
    float altitude_m;          // Calculated altitude in meters
} bmp280_data_t;

// --- Global Variables ---
extern bmp280_calib_t bmp280_calib;  // BMP280 calibration data
extern int32_t t_fine;               // Temperature fine resolution for pressure calculation

// --- Function Prototypes ---
/**
 * @brief Initialize IMU 
 * 
 * Configures I2C bus, wakes up the device and stuff
 * 
 * @return 0 on success, -1 on failure
 */
int imu_init();

/**
 * @brief Initialize I2C master interface and detect sensors
 * 
 * Configures I2C bus, checks for MPU9250 and BMP280 sensor presence
 * by reading their identification registers.
 * 
 * @return 0 on success, -1 on failure
 */
int i2c_master_init(void);

/**
 * @brief Read single byte from I2C device
 * 
 * @param dev_addr Device I2C address
 * @param reg_addr Register address to read from
 * @param data Pointer to store read data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Read multiple bytes from I2C device
 * 
 * @param dev_addr Device I2C address
 * @param reg_addr Starting register address
 * @param data Pointer to buffer for read data
 * @param len Number of bytes to read
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Combine high and low bytes into 16-bit signed value
 * 
 * @param high High byte
 * @param low Low byte
 * @return Combined 16-bit signed integer
 */
int16_t combine_bytes(uint8_t high, uint8_t low);

/**
 * @brief Initialize MPU9250 IMU sensor
 * 
 * Wakes up the device, configures accelerometer range (±2g),
 * and gyroscope range (±250°/s).
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mpu9250_init(void);

/**
 * @brief Read all sensor data from MPU9250
 * 
 * Reads accelerometer, gyroscope, and temperature data in a single
 * I2C transaction for efficiency.
 * 
 * @param data Pointer to mpu9250_data_t structure to store results
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t read_mpu9250_data(mpu9250_data_t *data);

/**
 * @brief Initialize BMP280 barometric sensor
 * 
 * Configures measurement modes, reads factory calibration coefficients,
 * and sets up continuous measurement operation.
 * 
 * @return ESP_OK on success, error code otherwise
 */


esp_err_t bmp280_init(void);

/**
 * @brief Compensate raw temperature using BMP280 calibration
 * 
 * Applies factory calibration coefficients to convert raw ADC values
 * to accurate temperature readings in Celsius.
 * 
 * @param adc_T Raw temperature ADC value
 * @return Temperature in degrees Celsius
 */
float bmp280_compensate_temperature(int32_t adc_T);

/**
 * @brief Compensate raw pressure using BMP280 calibration
 * 
 * Applies factory calibration coefficients to convert raw ADC values
 * to accurate pressure readings in hPa (hectopascals).
 * 
 * @param adc_P Raw pressure ADC value
 * @return Pressure in hPa
 */
float bmp280_compensate_pressure(int32_t adc_P);

/**
 * @brief Read all seto bmp280_data_t structure to store results
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t read_bmp280_data(bmp280_data_t *data);

// --- Conversion Functions ---

/**
 * @brief Convert raw accelerometer data to g-force
 * 
 * @param raw_accel Raw accelerometer value
 * @return Acceleration in g-force units
 */
float accel_to_g(int16_t raw_accel);

/**
 * @brief Convert raw gyroscope data to degrees per second
 * 
 * @param raw_gyro Raw gyroscope value
 * @return Angular velocity in degrees per second
 */
float gyro_to_dps(int16_t raw_gyro);

/**nsor data from BMP280
 * 
 * Reads pressure and temperature data, then applies calibration
 * for accurate measurements.
 * 
 * @param data Pointer 
 * @brief Convert raw MPU9250 temperature to Celsius
 * 
 * Note: MPU9250 temperature sensor may be inaccurate (±few degrees).
 * Use BMP280 temperature for accurate ambient temperature readings.
 * 
 * @param raw_temp Raw temperature value from MPU9250
 * @return Temperature in degrees Celsius
 */
float temp_to_celsius(int16_t raw_temp);

/**
 * @brief Alternative MPU9250 temperature conversion method
 * 
 * Uses alternative formula from MPU9250 datasheet.
 * 
 * @param raw_temp Raw temperature value from MPU9250
 * @return Temperature in degrees Celsius
 */
float temp_to_celsius_alt(int16_t raw_temp);

/**
 * @brief Calculate altitude from atmospheric pressure
 * 
 * Uses the international barometric formula to calculate altitude
 * based on atmospheric pressure, using standard sea level pressure
 * (1013.25 hPa) as reference.
 * 
 * @param pressure_hpa Current atmospheric pressure in hectopascals (hPa)
 * @return Altitude in meters above sea level
 */
float calculate_altitude(float pressure_hpa);

#ifdef __cplusplus
}
#endif

#endif // IMU_H