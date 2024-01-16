#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// MPU-6050 I2C address
#define MPU6050_ADDR 0x68

// MPU-6050 Registers
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// Function to initialize I2C
void i2c_init() {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
}

// Function to write a byte to the MPU-6050
void write_byte(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    i2c_write_blocking(i2c0, MPU6050_ADDR, buf, 2, false);
}

// Function to read bytes from the MPU-6050
void read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, MPU6050_ADDR, buffer, length, false);
}

// Function to read a 16-bit value
int16_t read_16bit(uint8_t reg) {
    uint8_t buffer[2];
    read_bytes(reg, buffer, 2);
    return (buffer[0] << 8) | buffer[1];
}

int main() {
    stdio_init_all();
    i2c_init();

    // Wake up the MPU-6050
    write_byte(PWR_MGMT_1, 0);

    while (true) {
        // Read accelerometer and gyroscope data
        int16_t ax = read_16bit(ACCEL_XOUT_H);
        int16_t gy = read_16bit(GYRO_XOUT_H);

        // Print the readings
        printf("Accel X: %d, Gyro Y: %d\n", ax, gy);

        sleep_ms(500);
    }

    return 0;
}
