#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include "global.h"
#include "driver/mcpwm.h"

#define SERVO_PIN 18  

void test_servo_config(void);
void servo_init();
void move_servo();

#endif // SERVO_CONFIG_H