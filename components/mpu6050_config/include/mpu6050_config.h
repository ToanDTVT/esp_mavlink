#ifndef MPU6050_CONFIG_H
#define MPU6050_CONFIG_H

#include "global.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

#define MPU6050_ADDR         0x68

void i2c_master_init();
esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t i2c_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len);
void mpu6050_init();
void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
void test_mpu6050_config(void);

#endif // MPU6050_CONFIG_H