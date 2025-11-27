#ifndef MOTORS_HPP
#define MOTORS_HPP

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define MAX_WRAP 1023

class Motor {
    public:
        Motor(uint pwm1_pin, uint pwm2_pin);
        void set_speed(int16_t speed);
    private:
        uint _pwm1_pin;
        uint _pwm2_pin;
        uint _slice_num1;
        uint _slice_num2;
};

class MotorL298N {
    public:
        MotorL298N(uint enable_pin, uint in1_pin, uint in2_pin);
        void set_speed(int16_t speed);
    private:
        uint _enable_pin;
        uint _in1_pin;
        uint _in2_pin;
        uint _slice_num;
};

#endif // MOTORS_HPP