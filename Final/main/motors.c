
#include "motors.h"

static const char *TAG = "Servo_control";

// Calculate duty cycle for servo position (0-180 degrees)
uint32_t servo_angle_to_duty(uint8_t angle) {
    // Map angle (0-180) to pulse width (500-2100 microseconds)
    uint32_t pulse_width = SERVO_MIN_PULSE_WIDTH + (angle * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH)) / 180;
    
    // Convert pulse width to duty cycle
    // PWM period = 1/50Hz = 20ms = 20000 microseconds
    // Duty cycle = (pulse_width / 20000) * (2^16 - 1) for 16-bit resolution
    uint32_t duty = (pulse_width * ((1 << 16) - 1)) / 20000;
    return duty;
}



int configure_servos(void)
{
    ESP_LOGI(TAG, "Configuring dual servo control!");
    
    // Configure the timer for both servos
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,  // Use low-speed mode for better stability
        .timer_num        = LEDC_TIMER_0,         // Timer 0 for both servos
        .duty_resolution  = PWM_RESOLUTION,       // 16-bit resolution
        .freq_hz          = PWM_FREQUENCY,        // 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };

    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer");
        return -1;
    }

    // Configure channel for Servo 1 (GPIO 13)
    ledc_channel_config_t servo1_channel = {
        .gpio_num       = SERVO1_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = servo_angle_to_duty(90), // Start at center position (90 degrees)
        .hpoint         = 0
    };
    if (ledc_channel_config(&servo1_channel) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Servo 1 channel");
        return -1;
    }

    // Configure channel for Servo 2 (GPIO 14)
    ledc_channel_config_t servo2_channel = {
        .gpio_num       = SERVO2_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = servo_angle_to_duty(90), // Start at center position (90 degrees)
        .hpoint         = 0
    };

    if (ledc_channel_config(&servo2_channel) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Servo 2 channel");
        return -1;
    }

    ESP_LOGI(TAG, "Servo 1 configured on GPIO %d", SERVO1_PIN);
    ESP_LOGI(TAG, "Servo 2 configured on GPIO %d", SERVO2_PIN);
}

// Global variables to track current servo positions
static uint8_t servo1_current_angle = 90;
static uint8_t servo2_current_angle = 90;

// Function to move a specific servo to an angle
void set_servo_angle(uint8_t servo_num, uint8_t angle) {
    if (angle > 180) {
        angle = 180; // Clamp to maximum angle
    }
    
    uint32_t duty = servo_angle_to_duty(angle);
    ledc_channel_t channel = (servo_num == 1) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
    
    // Update current position tracking
    if (servo_num == 1) {
        servo1_current_angle = angle;
    } else {
        servo2_current_angle = angle;
    }
    
    ESP_LOGI(TAG, "Servo %d moved to %d degrees (duty: %lu)", servo_num, angle, duty);
}

// Simple servo control functions
void balloon_release(void) {
    set_servo_angle(1, 0);  // Open position (0 degrees)
    ESP_LOGI(TAG, "Servo 1 OPENED");
}

void balloon_lock(void) {
    set_servo_angle(1, 180);  // Close position (180 degrees)
    ESP_LOGI(TAG, "Servo 1 CLOSED");
}

void parachute_deploy(void) {
    set_servo_angle(2, 0);  // Open position (0 degrees)
    ESP_LOGI(TAG, "Servo 2 OPENED");
}

void parachute_lock(void) {
    set_servo_angle(2, 180);  // Close position (180 degrees)
    ESP_LOGI(TAG, "Servo 2 CLOSED");
}

void move_servo_task(void *pvParameter){
    // Configure both servos
    configure_servos();
    
    ESP_LOGI(TAG, "Starting simple servo open/close demo");
    
    // Wait a moment for servos to initialize
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    while (1) {
        // Test Servo 1
        ESP_LOGI(TAG, "Payload detached from balloon.");
        balloon_release();
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 2 seconds
        
        ESP_LOGI(TAG, "Payload secured to balloon.");
        parachute_lock();
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 2 seconds
        
        // Test Servo 2
        ESP_LOGI(TAG, "Parachute deployed");
        parachute_deploy();
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 2 seconds
        
        ESP_LOGI(TAG, "Parachute locked to payload");
        parachute_lock();
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 2 seconds
        
        // Test both together
        ESP_LOGI(TAG, "Testing both servos together...");
        balloon_release();
        parachute_deploy();
        vTaskDelay(3000 / portTICK_PERIOD_MS);  // Wait 3 seconds
        
        parachute_lock();
        parachute_lock();
        vTaskDelay(3000 / portTICK_PERIOD_MS);  // Wait 3 seconds
        
        ESP_LOGI(TAG, "Demo cycle complete. Repeating in 2 seconds...");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}


//void app_main(void)
//{

    /* Configure the peripheral according to the LED type */
    //xTaskCreate(led_task, "LedTask", 2048, NULL, 1, NULL);
   // xTaskCreate(move_servo_task, "MoveServoTask", 2048, NULL, 1, NULL);
//}
