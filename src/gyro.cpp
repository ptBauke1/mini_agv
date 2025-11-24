#include "gyro.hpp"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

Gyro::Gyro(i2c_inst_t* i2c, uint8_t address, uint scl_pin, uint sda_pin) : _i2c(i2c), _address(address), _scl_pin(scl_pin), _sda_pin(sda_pin) {
    i2c_init(_i2c, 400 * 1000); // Initialize I2C at 400kHz
    gpio_set_function(_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(_sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(_scl_pin);
    gpio_pull_up(_sda_pin);
}

void Gyro::reset() {
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(_i2c, _address, buf, 2, false);

    sleep_ms(100);
    
    buf[1] = 0x00;
    i2c_write_blocking(_i2c, _address, buf, 2, false);
    sleep_ms(10);
}

void Gyro::calibrate() {
    const int samples = 200;  // Reduced from 2000 to 200 (2 seconds instead of 20)
    int32_t sum[3] = {0, 0, 0};

    for (int i = 0; i < samples; i++) {
        read_linear_acceleration();
        for (int j = 0; j < 3; j++) {
            sum[j] += _accel[j];
        }
        sleep_ms(10);
    }

    for (int j = 0; j < 3; j++) {
        _accel_offset[j] = sum[j] / samples;
    }
}

void Gyro::read_linear_acceleration() {
    uint8_t reg = 0x3B; // Starting register for accelerometer data
    uint8_t data[6];

    int result = i2c_write_timeout_us(_i2c, _address, &reg, 1, true, 10000);  // 10ms timeout
    if (result < 0) return;  // I2C error, skip reading
    
    result = i2c_read_timeout_us(_i2c, _address, data, 6, false, 10000);  // 10ms timeout
    if (result < 0) return;  // I2C error, skip reading
    
    for (int i = 0; i < 3; i++) {
        _accel[i] = (data[2*i] << 8) | data[2*i + 1];
    }
}

int16_t Gyro::get_accel_x() {
    return _accel[X_AXIS] - _accel_offset[X_AXIS];
}

int16_t Gyro::get_speed_x() {
    static int32_t speed_x = 0;
    speed_x += get_accel_x();
    return speed_x;
}

int16_t Gyro::get_position_x() {
    static int32_t position_x = 0;
    position_x += get_speed_x();
    return position_x;
}

int16_t Gyro::get_accel_y() {
    return _accel[Y_AXIS] - _accel_offset[Y_AXIS];
}

int16_t Gyro::get_speed_y() {
    static int32_t speed_y = 0;
    speed_y += get_accel_y();
    return speed_y;
}

int16_t Gyro::get_position_y() {
    static int32_t position_y = 0;
    position_y += get_speed_y();
    return position_y;
}