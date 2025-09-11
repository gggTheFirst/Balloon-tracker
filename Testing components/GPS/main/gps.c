/*
 * gps.c
 * Example code to test communication between ESP32 and u-blox MAX-M10S GPS module over UART.
 * RX (ESP32) - GPIO12
 * TX (ESP32) - GPIO13
 *
 * This code initializes UART, receives NMEA sentences from the GPS, and prints them to the serial monitor.
 * 
 * Functions explained:
 * - uart_driver_install: Installs UART driver and allocates buffers.
 * - uart_param_config: Configures UART parameters (baud rate, data bits, parity, stop bits, flow control).
 * - uart_set_pin: Assigns UART pins for TX, RX, RTS, CTS.
 * - uart_read_bytes: Reads bytes from UART.
 */

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define UART_NUM            UART_NUM_2         // Use UART2
#define UART_TX_PIN         (17)               // ESP32 TX pin connected to GPS RX
#define UART_RX_PIN         (16)               // ESP32 RX pin connected to GPS TX
#define UART_BAUD_RATE      (9600)             // Default baud rate for MAX-M10S
#define UART_BUF_SIZE       (1024)             // Buffer size for incoming data

static const char *TAG = "GPS_TEST";

void app_main(void)
{
    // UART configuration structure
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,         // 8 data bits
        .parity    = UART_PARITY_DISABLE,      // No parity
        .stop_bits = UART_STOP_BITS_1,         // 1 stop bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // No hardware flow control
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters
    uart_param_config(UART_NUM, &uart_config);

    // Set UART pins (TX, RX, RTS, CTS)
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver with RX buffer
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    uint8_t data[UART_BUF_SIZE];

    ESP_LOGI(TAG, "GPS UART test started. Waiting for NMEA sentences...");

    while (1) {
        // Read bytes from UART
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate for printing
            // Print received data (NMEA sentences)
            ESP_LOGI(TAG, "Received: %s", (char *)data);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to avoid flooding
    }
}

/*
 * How it works:
 * - The ESP32 UART1 is configured to communicate with the GPS module at 9600 baud.
 * - The RX and TX pins are set to GPIO1 and GPIO3.
 * - The code continuously reads data from the GPS and prints it to the serial monitor.
 * - You should see NMEA sentences (starting with $G...) if everything is connected correctly.
 */