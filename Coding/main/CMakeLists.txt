set(main_priv_requires
    esp_driver_gpio
    esp_driver_ledc
    esp_driver_uart
    esp32-camera
    esp_psram
    esp_timer
    spi_flash)

idf_component_register(SRCS "motor_control.c" "camera_tracker.c" "serial_comm.c" "hello_world_main.c"
                       PRIV_REQUIRES ${main_priv_requires}
                       INCLUDE_DIRS "")
