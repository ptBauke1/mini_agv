#include "motors.hpp"

Motor::Motor(uint pwm1_pin, uint pwm2_pin)
    : _pwm1_pin(pwm1_pin), _pwm2_pin(pwm2_pin) {
    gpio_set_function(_pwm1_pin, GPIO_FUNC_PWM);
    gpio_set_function(_pwm2_pin, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    // pwm config -> 1023 wrap and 20kHz frequency
    // base clock -> 125MHz
    pwm_config_set_wrap(&config, MAX_WRAP);
    pwm_config_set_clkdiv(&config, 6.1f); // 125MHz / 6.1 / 1024 = ~20kHz
    _slice_num1 = pwm_gpio_to_slice_num(_pwm1_pin);
    _slice_num2 = pwm_gpio_to_slice_num(_pwm2_pin);
    pwm_init(_slice_num1, &config, true);
    pwm_init(_slice_num2, &config, true);
}

void Motor::set_speed(int16_t speed) {
    if (speed > MAX_WRAP) {
        speed = MAX_WRAP;
    } else if (speed < -MAX_WRAP) {
        speed = -MAX_WRAP;
    }
    if (speed >= 0) {
        pwm_set_gpio_level(_pwm1_pin, speed);
        pwm_set_gpio_level(_pwm2_pin, 0);
    } else {
        pwm_set_gpio_level(_pwm1_pin, 0);
        pwm_set_gpio_level(_pwm2_pin, -speed);
    }
}

MotorL298N::MotorL298N(uint enable_pin, uint in1_pin, uint in2_pin)
    : _enable_pin(enable_pin), _in1_pin(in1_pin), _in2_pin(in2_pin) {
    gpio_set_function(_enable_pin, GPIO_FUNC_PWM);
    gpio_init(_in1_pin);
    gpio_set_dir(_in1_pin, GPIO_OUT);
    gpio_init(_in2_pin);
    gpio_set_dir(_in2_pin, GPIO_OUT);

    pwm_config config = pwm_get_default_config();
    // pwm config -> 1023 wrap and 20kHz frequency
    // base clock -> 125MHz
    pwm_config_set_wrap(&config, MAX_WRAP);
    pwm_config_set_clkdiv(&config, 6.1f); // 125MHz / 6.1 / 1024 = ~20kHz
    _slice_num = pwm_gpio_to_slice_num(_enable_pin);
    pwm_init(_slice_num, &config, true);
}

void MotorL298N::set_speed(int16_t speed) {
    if (speed > MAX_WRAP) {
        speed = MAX_WRAP;
    } else if (speed < -MAX_WRAP) {
        speed = -MAX_WRAP;
    }
    if (speed >= 0) {
        gpio_put(_in1_pin, 1);
        gpio_put(_in2_pin, 0);
        pwm_set_gpio_level(_enable_pin, speed);
    } else {
        gpio_put(_in1_pin, 0);
        gpio_put(_in2_pin, 1);
        pwm_set_gpio_level(_enable_pin, -speed);
    }
}