#ifndef MOTORS_H
#define MOTORS_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- Servo Configuration Constants ---
#define SERVO1_PIN     13  // GPIO pin for servo 1
#define SERVO2_PIN     14  // GPIO pin for servo 2
#define PWM_FREQUENCY  50  // 50 Hz for servo control (standard servo frequency)
#define PWM_RESOLUTION LEDC_TIMER_16_BIT // 16-bit resolution for better servo control

// Servo pulse width constants (in microseconds)
#define SERVO_MIN_PULSE_WIDTH 500   // 0.5ms (0 degrees)
#define SERVO_MAX_PULSE_WIDTH 2100  // 2.1ms (180 degrees)
#define SERVO_CENTER_PULSE_WIDTH 1500 // 1.5ms (90 degrees)

// --- Function Prototypes ---


int configure_servos(void);

void set_servo_angle(uint8_t servo_num, uint8_t angle);
uint32_t servo_angle_to_duty(uint8_t angle);

// Open 0 degree; close 180 degrees
void balloon_release(void);
void balloon_lock(void);

// Open 0 degree; close 180 degrees
void parachute_deploy(void);
void parachute_lock(void);

/**
 * @brief Servo movement demo task
 * 
 * FreeRTOS task that demonstrates servo open/close operations.
 * This function runs continuously and cycles through servo movements.
 * 
 * @param pvParameter Task parameter (unused)
 */
void move_servo_task(void *pvParameter);

#ifdef __cplusplus
}
#endif

#endif // MOTORS_H