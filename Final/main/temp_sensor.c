
#include "temp_sensor.h"           

adc_oneshot_unit_handle_t adc1_handle;

// Converts raw ADC reading to Celsius temperature
float get_temperature(int adc_reading) {
    float voltage = ((float)adc_reading / 4095.0) * 3.3; // Convert ADC value to voltage
    float kelvin = voltage / 0.01; // LM335: 10mV per Kelvin
    float celsius = kelvin - 273.15; // Convert Kelvin to Celsius
    return celsius;
}

// Performs multiple ADC readings and returns the average
int get_averaged_adc_reading(adc_channel_t channel) {
    int sum = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        int adc_reading = 0;
        esp_err_t ret = adc_oneshot_read(adc1_handle, channel, &adc_reading);
        
        if (ret == ESP_OK) {
            sum += adc_reading;
            valid_readings++;
        }
        
        // Small delay between readings to allow ADC to settle
        if (i < ADC_SAMPLE_COUNT - 1) {
            vTaskDelay(ADC_SAMPLE_DELAY_MS / portTICK_PERIOD_MS);
        }
    }
    
    // Return average if we have valid readings, otherwise return 0
    return (valid_readings > 0) ? (sum / valid_readings) : 0;
}


int temp_sensor_configure_adc() {
   // --- 1. Create ADC oneshot unit handle ---
    // This handle lets you use ADC1 in oneshot mode (single conversion per call)
   
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,           // Use ADC1 hardware
        .ulp_mode = ADC_ULP_MODE_DISABLE // Don't use ULP co-processor
    };
    esp_err_t err;
    err = adc_oneshot_new_unit(&init_cfg, &adc1_handle); // Initialize ADC unit
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC1");
        return -1;
    }
    // --- 2. Configure ADC channels ---
    // Set resolution and attenuation for each channel (sensor)
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_WIDTH,           // 12-bit resolution
        .atten = ADC_ATTEN               // 12dB attenuation (max voltage)
    };
    err = adc_oneshot_config_channel(adc1_handle, TEMP_SENSOR_1, &chan_cfg);    // Sensor 1
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TEMP_SENSOR_1");
        return -1;
    }
    err = adc_oneshot_config_channel(adc1_handle, TEMP_SENSOR_2, &chan_cfg);  // Sensor 2
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TEMP_SENSOR_2");
        return -1;
    }

    return 0; // Success
}

float measure_inside_temperature() {
    int adc_reading = get_averaged_adc_reading(TEMP_SENSOR_1);
    return get_temperature(adc_reading);
}
float measure_outside_temperature() {
    int adc_reading = get_averaged_adc_reading(TEMP_SENSOR_2);
    return get_temperature(adc_reading);
}
void app_main() {
    temp_sensor_configure_adc(); // Initialize ADC
    
    // --- 3. Main loop: read sensors and print temperature ---
    while (1) {
        // Get averaged ADC readings from both sensors
        int adc_reading1 = get_averaged_adc_reading(TEMP_SENSOR_1);
        int adc_reading2 = get_averaged_adc_reading(TEMP_SENSOR_2);

        // Convert averaged ADC readings to Celsius
        float temp1 = measure_inside_temperature();
        float temp2 = measure_outside_temperature();

        // Print results to serial monitor
        printf("Sensor 1 (GPIO34): %.2f°C (avg of %d samples)\n", temp1, ADC_SAMPLE_COUNT);
        printf("Sensor 2 (GPIO35): %.2f°C (avg of %d samples)\n", temp2, ADC_SAMPLE_COUNT);

        // Wait 1 second before next reading
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
