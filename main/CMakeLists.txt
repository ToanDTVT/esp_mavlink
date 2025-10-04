idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS "." "../components" 
                       REQUIRES global gpio_config servo_config mpu6050_config control_pixhawk mavlink driver esp_rom esp_common esp_system nvs_flash esp_event esp_http_client esp_https_ota lwip protocol_examples_common esp_netif json app_update mqtt)
