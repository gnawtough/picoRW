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
    i2c_init(i2c0, 400 * 1000); // Initialize I2C communication
    gpio_set_function(4, GPIO_FUNC_I2C); // Set GPIO 4 to I2C function 6050
    gpio_set_function(5, GPIO_FUNC_I2C); // Set GPIO 5 to I2C function 6050
    gpio_pull_up(4); // Enable internal pull-up resistor on GPIO 4 6050
    gpio_pull_up(5); // Enable internal pull-up resistor on GPIO 5 6050
}

// Function to write a byte to the MPU-6050
void write_byte(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    i2c_write_blocking(i2c0, MPU6050_ADDR, buf, 2, false);
}

// Function to read bytes from a generic device over I2C
void read_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(i2c0, addr, &reg, 1, true); // Write the register address to read from
    i2c_read_blocking(i2c0, addr, buffer, length, false); // Read the data into buffer
}

// Reading a 16-bit value from MPU-6050
int16_t read_16bit_mpu6050(uint8_t reg) {
    uint8_t buffer[2];
    read_bytes(MPU6050_ADDR, reg, buffer, 2); // Updated call with MPU6050_ADDR
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
        // Read accelerometer data for all axes
        int16_t ax = read_16bit_mpu6050(ACCEL_XOUT_H);
        int16_t ay = read_16bit_mpu6050(ACCEL_XOUT_H + 2);
        int16_t az = read_16bit_mpu6050(ACCEL_XOUT_H + 4);

        // Read gyroscope data for all axes
        int16_t gx = read_16bit_mpu6050(GYRO_XOUT_H);
        int16_t gy = read_16bit_mpu6050(GYRO_XOUT_H + 2);
        int16_t gz = read_16bit_mpu6050(GYRO_XOUT_H + 4);

        // Conversion data 
        float ax_g = ax / 16384.0; // Converts accel data based on default +-2g setting, scale: 16384 LSB/g force
        float ay_g = ay / 16384.0;
        float az_g = az / 16384.0;

        float gx_dps = gx / 131.0; // Convert gyro data, scale factor +- 250dps, 131 LSB/(deg/s)
        float gy_dps = gy / 131.0; 
        float gz_dps = gz / 131.0;

         // Print the readings
        printf("Accel (g): X=%.2f, Y=%.2f, Z=%.2f, Gyro (degrees/sec): X=%.2f, Y=%.2f, Z=%.2f\n", 
               ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);

        // Play a tone based on accelerometer X value
        if (ax_g > 0.99) {
            play_tone(800, 100); // Play a 440 Hz tone for 100 ms
        }

        sleep_ms(500);
    }

    return 0;
}
