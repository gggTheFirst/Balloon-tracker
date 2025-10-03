#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "motors.h"

static const char *TAG = "UART_ECHO";

// UART configuration
#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_DATA_8_BITS   UART_DATA_8_BITS
#define UART_PARITY_DISABLE UART_PARITY_DISABLE
#define UART_STOP_BITS_1   UART_STOP_BITS_1
#define UART_FLOW_CTRL_DISABLE UART_HW_FLOWCTRL_DISABLE
#define UART_SOURCE_CLK    UART_SCLK_DEFAULT

#define BUF_SIZE (1024)

// Initialize UART
void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_FLOW_CTRL_DISABLE,
        .source_clk = UART_SOURCE_CLK,
    };
    
    // Install UART driver, and get the queue
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    ESP_LOGI(TAG, "UART initialized on port %d at %d baud", UART_PORT_NUM, UART_BAUD_RATE);
}

// UART echo task
void uart_echo_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    
    ESP_LOGI(TAG, "UART Echo Task Started");
    ESP_LOGI(TAG, "Type something and press Enter - I'll echo it back!");
    ESP_LOGI(TAG, "====================================================");
    
    while (1) {
        // Read data from UART
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            // Write data back to UART (echo)
            uart_write_bytes(UART_PORT_NUM, (const char *) data, len);
            
            if (data[0] == 'q'){
                balloon_release();
                parachute_deploy();

            }else if (data[0] == 'e'){
                balloon_lock();
                parachute_lock();
            }
            // Null-terminate for logging
            data[len] = '\0';
            
            // Log what was received (optional - remove if you don't want extra output)
            ESP_LOGI(TAG, "Received %d bytes: '%s'", len, (char*)data);
            
            // Add a newline after echo if the received data doesn't end with one
            if (len > 0 && data[len-1] != '\n') {
                uart_write_bytes(UART_PORT_NUM, "\n", 1);
            }
        }
        
        // Small delay to prevent watchdog issues
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    free(data);
}

void app_main(void)
{
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          UART ECHO PROGRAM            ║");
    ESP_LOGI(TAG, "║        Starting Up...                 ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
    // Initialize UART
    uart_init();
    configure_servos();
    // Create echo task
    xTaskCreate(uart_echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "🎯 UART Echo ready!");
    ESP_LOGI(TAG, "💬 Send me anything and I'll echo it back!");
}