#include "servo_config.h"


// Hàm chuyển góc thành duty
static uint32_t angle_to_pulsewidth(float angle) {
    // SG90: 0.5ms -> 0°, 2.5ms -> 180°
    uint32_t min_pulsewidth_us = 500;   // 0.5ms
    uint32_t max_pulsewidth_us = 2500;  // 2.5ms
    uint32_t max_angle = 180;

    return min_pulsewidth_us + (angle * (max_pulsewidth_us - min_pulsewidth_us) / max_angle);
}

void servo_init()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN); // Chân PWM0A

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    // Tần số 50Hz cho servo
    pwm_config.cmpr_a = 0;        // Duty cycle ban đầu 0%
    pwm_config.cmpr_b = 0;        // Không sử dụng kênh B
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void move_servo()
{
   for (int angle = 0; angle <= 180; angle += 10) {
            uint32_t pulsewidth = angle_to_pulsewidth(angle);
            ESP_LOGI("SERVO", "Angle: %d, Pulse: %dus", angle, pulsewidth);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulsewidth);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

}

void test_servo_config(void)
{
    printf("This is a test function from servo_config component.\n");
}