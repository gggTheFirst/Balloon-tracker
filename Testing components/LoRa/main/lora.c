// Minimal LoRa send/receive example for ESP-IDF (SX1278)
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define PIN_NUM_MISO 33
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  26
#define PIN_NUM_CS   4
#define PIN_NUM_RST  25
#define PIN_NUM_DIO0 32

spi_device_handle_t spi;

void lora_reset() {
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

uint8_t lora_read_reg(uint8_t reg) {
    uint8_t tx[2] = {reg & 0x7F, 0x00};
    uint8_t rx[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(spi, &t);
    return rx[1];
}

void lora_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg | 0x80, val};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };
    spi_device_transmit(spi, &t);
}

void lora_send_packet(const uint8_t* data, uint8_t len) {
    // Set to standby
    lora_write_reg(0x01, 0x81);
    // Set FIFO addr ptr
    lora_write_reg(0x0D, 0x00);
    // Write payload to FIFO
    for (uint8_t i = 0; i < len; i++) {
        lora_write_reg(0x00, data[i]);
    }
    // Set payload length
    lora_write_reg(0x22, len);
    // Set to TX mode
    lora_write_reg(0x01, 0x83);
    // Wait for TX done
    while ((lora_read_reg(0x12) & 0x08) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Clear IRQ
    lora_write_reg(0x12, 0x08);
}

uint8_t lora_receive_packet(uint8_t* buf, uint8_t maxlen) {
    // Set to RX continuous
    lora_write_reg(0x01, 0x85);
    // Wait for RX done
    while ((lora_read_reg(0x12) & 0x40) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Get length
    uint8_t len = lora_read_reg(0x13);
    if (len > maxlen) len = maxlen;
    // Set FIFO addr ptr to current RX addr
    lora_write_reg(0x0D, lora_read_reg(0x10));
    // Read payload
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = lora_read_reg(0x00);
    }
    // Clear IRQ
    lora_write_reg(0x12, 0x40);
    return len;
}

void lora_init() {
    // Set sleep mode
    lora_write_reg(0x01, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Set LoRa mode
    lora_write_reg(0x01, 0x81);
    // Set frequency to 434 MHz (example)
    uint32_t frf = (uint32_t)((434000000.0 / 61.03515625));
    lora_write_reg(0x06, (frf >> 16) & 0xFF);
    lora_write_reg(0x07, (frf >> 8) & 0xFF);
    lora_write_reg(0x08, frf & 0xFF);
    // Set output power to max
    lora_write_reg(0x09, 0x8F);
    // Set base addresses
    lora_write_reg(0x0E, 0x00);
    lora_write_reg(0x0F, 0x00);
    // Set LNA boost
    lora_write_reg(0x0C, 0x23);
    // Set spreading factor SF7
    lora_write_reg(0x1E, 0x74);
    // Set bandwidth 125 kHz
    lora_write_reg(0x1D, 0x72);
}

void app_main(void) {
    // Configure pins
    gpio_reset_pin(PIN_NUM_RST);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_NUM_CS);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);
    lora_reset();

    // SPI bus config
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(HSPI_HOST, &buscfg, 1);

    // SPI device config
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi);

    lora_init();

    printf("LoRa SX1278 simple send/receive test\n");

    // Example: send a packet every 5 seconds, receive in between
    uint8_t rxbuf[32];
    while (1) {
        const char* msg = "Hello LoRa!";
        printf("Sending: %s\n", msg);
        lora_send_packet((const uint8_t*)msg, 11);
        printf("Waiting for packet...\n");
        uint8_t len = lora_receive_packet(rxbuf, sizeof(rxbuf));
        printf("Received (%d bytes): ", len);
        for (int i = 0; i < len; i++) putchar(rxbuf[i]);
        putchar('\n');
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}