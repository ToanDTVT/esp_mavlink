#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include "global.h"
#include "driver/gpio.h"
#include "control_pixhawk.h"

#define BUTTON_GPIO    GPIO_NUM_21

void test_gpio_config(void);
void create_button_task();
void press_button();
void gpio_init(void);

#endif // GPIO_CONFIG_H