#include "mpu6050_config.h"


void i2c_master_init() {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


// Hàm ghi 1 byte vào MPU6050
esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data) {
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, &((uint8_t){reg_addr, data}), 2, 1000 / portTICK_PERIOD_MS);
}

// Hàm đọc nhiều byte từ MPU6050
esp_err_t i2c_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// Hàm init MPU6050
void mpu6050_init() {
    i2c_write_byte(0x6B, 0x00); // Wake up MPU6050
}

// Hàm đọc gia tốc
void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6];
    i2c_read_bytes(0x3B, buf, 6);
    *ax = (buf[0] << 8) | buf[1];
    *ay = (buf[2] << 8) | buf[3];
    *az = (buf[4] << 8) | buf[5];
}


void test_mpu6050_config(void)
{
    printf("This is a test function from mpu6050_config component.\n");
}