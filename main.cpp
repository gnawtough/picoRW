#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// MPU-6050 I2C address
#define MPU6050_ADDR 0x68

// MPU-6050 Registers
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// Speaker GPIO
#define SPEAKER_PIN 0

// Function to initialize I2C
void init_i2c() {
    i2c_init(i2c0, 100 * 1000); // Initialize I2C communication
    gpio_set_function(4, GPIO_FUNC_I2C); // Set GPIO 4 to I2C function
    gpio_set_function(5, GPIO_FUNC_I2C); // Set GPIO 5 to I2C function
    gpio_pull_up(4); // Enable internal pull-up resistor on GPIO 4
    gpio_pull_up(5); // Enable internal pull-up resistor on GPIO 5
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

// Function to play sound
void play_tone(uint frequency, uint duration_ms) {
    uint half_period = 1000000 / frequency / 2;
    uint64_t end_time = to_us_since_boot(get_absolute_time()) + duration_ms * 1000;

    while (to_us_since_boot(get_absolute_time()) < end_time) {
        gpio_put(SPEAKER_PIN, 1);
        sleep_us(half_period);
        gpio_put(SPEAKER_PIN, 0);
        sleep_us(half_period);
    }
}


int main() {
    stdio_init_all();
    init_i2c();

    // Init speaker pin
    gpio_init(SPEAKER_PIN);
    gpio_set_dir(SPEAKER_PIN, GPIO_OUT);

    // Wake up the MPU-6050
    write_byte(PWR_MGMT_1, 0);

    while (true) {
        // Read accelerometer and gyroscope data
        int16_t ax = read_16bit(ACCEL_XOUT_H);
        int16_t gy = read_16bit(GYRO_XOUT_H);

        // Print the readings
        printf("Accel X: %d, Gyro Y: %d\n", ax, gy);

        // Play a tone based on accelerometer X value
        if (ax > threshold_value) {
            play_tone(440, 100); // Play a 440 Hz tone for 100 ms
        }

        sleep_ms(500);
    }

    return 0;
}
