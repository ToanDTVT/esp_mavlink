/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "global.h"
#include "gpio_config.h"
#include "servo_config.h"
#include "mpu6050_config.h"
#include "control_pixhawk.h"
#include "mavlink/common/mavlink.h"

QueueHandle_t button_evt_queue;

void app_main(void)
{
    printf("Hello world!\n");
    uart_init();
    i2c_master_init();
    mpu6050_init();
    gpio_init();
    servo_init();

    button_evt_queue = xQueueCreate(5, sizeof(int));
    
    test_global();
    test_control_pixhawk();
    test_gpio_config();
    test_servo_config();
    test_mpu6050_config();
    

    create_mavlink_receive_task();
    create_button_task();
    create_disarm_task(); 

    while (1)
    {
        /* code */
        
        // send_heartbeat();
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // printf("\n");
        // move_servo();
        // send_disarm();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        // printf("\n");

    }
    
}




