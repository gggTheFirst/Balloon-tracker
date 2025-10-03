#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"      // FreeRTOS for delays
#include "freertos/task.h"         // FreeRTOS task functions
#include "esp_adc/adc_oneshot.h"   // New ESP-IDF ADC oneshot driver

#ifdef __cplusplus
extern "C" {
#endif

// --- ADC Channel Definitions ---
#define TEMP_SENSOR_1 ADC_CHANNEL_6 // ADC channel for GPIO34 (Sensor 1)
#define TEMP_SENSOR_2 ADC_CHANNEL_7 // ADC channel for GPIO35 (Sensor 2)

// --- ADC Configuration Constants ---
#define ADC_WIDTH ADC_BITWIDTH_12   // 12-bit resolution (0-4095)
#define ADC_ATTEN ADC_ATTEN_DB_12   // Attenuation: allows measuring up to ~3.3V
#define ADC_SAMPLE_COUNT 10         // Number of samples to average
#define ADC_SAMPLE_DELAY_MS 10      // Delay between samples in milliseconds

extern adc_oneshot_unit_handle_t adc1_handle;

// --- Function Prototypes ---
int temp_sensor_configure_adc(void);

// use 12 bit adc value to get temperature
float get_temperature(int adc_reading);

// polls multiple ADC readings and return the average
int get_averaged_adc_reading(adc_channel_t channel);

float measure_inside_temperature(void);

float measure_outside_temperature(void);

#ifdef __cplusplus
}
#endif

#endif // TEMP_SENSOR_H