#ifndef GYRO_HPP
#define GYRO_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

enum axes {
    X_AXIS = 0,
    Y_AXIS = 1,
    Z_AXIS = 2
};

class Gyro {
    private:
        i2c_inst_t* _i2c;
        uint8_t _address;
        uint _scl_pin;
        uint _sda_pin;
        int16_t _accel[3];
        int16_t _accel_offset[3];
        int16_t _last_x_position = 0;
        int16_t _last_y_position = 0;

    public:
        Gyro(i2c_inst_t* i2c, uint8_t address, uint scl_pin, uint sda_pin);
        void reset();
        void read_linear_acceleration();
        void calibrate();
        int16_t get_accel_x();
        int16_t get_accel_y();
        int16_t get_position_x();
        int16_t get_position_y();
        int16_t get_speed_x();
        int16_t get_speed_y();
};

#endif // GYRO_HPP