/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/ledc.h" // for pwm stuff


static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 2

#define PWM_PIN        26  // GPIO pin for PWM output
#define PWM_FREQUENCY  1000 // 5 kHz PWM frequency
#define PWM_RESOLUTION LEDC_TIMER_8_BIT // Resolution: 8 bits (0-255 duty)


static uint8_t s_led_state = 0;

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void configure_servo(void)
{
    ESP_LOGI(TAG, "Example configured to control servo!");
    // 1. Configure the timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE, // High-speed mode
        .timer_num        = LEDC_TIMER_0,         // Timer 0
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 2. Configure the channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = PWM_PIN,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Start with duty cycle = 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    // 3. Set duty cycle (example: 50% duty)
    int duty = 128; // (255 max for 8-bit resolution)
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);

    printf("PWM running on GPIO %d\n", PWM_PIN);

}

void move_servo_task(void *pvParameter){
    //code to move servo
    configure_servo();

    while (1) {
        ESP_LOGI("MOTOR", "Hello from move_servo_task!");
        // Add code to move the servo here
        ESP_LOGI(TAG, "Moving the servo to position!");
        // Example: Change duty cycle to move servo
        for (int duty = 0; duty <= 255; duty += 51) { // Move from 0% to 100% in steps
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

}

void led_task(void *pvParameter){
    //code to blink led
    while (1) {
        ESP_LOGI("LED", "Hello from led_task!");
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    xTaskCreate(led_task, "LedTask", 2048, NULL, 1, NULL);
    xTaskCreate(move_servo_task, "MoveServoTask", 2048, NULL, 1, NULL);
}
