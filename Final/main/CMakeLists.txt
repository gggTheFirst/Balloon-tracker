idf_component_register(SRCS "imu.c" "temp_sensor.c" "GPS.c" "balloon_tracker.c" "uart_echo.c" "motors.c" "landzone.c" "flight_box.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash esp_driver_ledc esp_driver_i2c esp_driver_uart
                       PRIV_REQUIRES esp_adc)
