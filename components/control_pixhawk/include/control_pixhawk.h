#ifndef CONTROL_PIXHAWK_H
#define CONTROL_PIXHAWK_H

#include "global.h"
#include "mavlink/common/mavlink.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_NUM_PIXHAWK            UART_NUM_1
#define TX_PIN_PIXHAWK              GPIO_NUM_17
#define RX_PIN_PIXHAWK              GPIO_NUM_16
#define UART_BUF_SIZE       1024


#define ESP_SYSTEM_ID      200   // ID riêng cho ESP32
#define ESP_COMPONENT_ID   MAV_COMP_ID_ONBOARD_COMPUTER

void test_control_pixhawk(void);
void uart_init (void);
void create_mavlink_receive_task();
void send_disarm ();
void send_heartbeat(void);
void create_disarm_task();



#endif // CONTROL_PIXHAWK_H 