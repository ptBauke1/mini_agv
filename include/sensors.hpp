#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define NUM_SENSORS 8
#define SENSOR_OFFSET 1000.0f*(NUM_SENSORS-1) / 2.0f

#define TIMEOUT_TIME 38000 // 38ms timeout for ultrasonic (max range ~4m)

class SensorArray {
    public:
        SensorArray(uint bit0, uint bit1, uint bit2, uint adc);
        void read_sensors();
        uint16_t get_sensor_value(uint8_t index);
        void print_values();
        float calculate_error();
        void calibrate();
    private:
        uint _bit_0_pin;
        uint _bit_1_pin;
        uint _bit_2_pin;
        uint _adc_pin;
        uint16_t _sensor_array[NUM_SENSORS];
        void _set_mux_channel(uint8_t channel);
        uint16_t _read_adc();
        uint16_t _max_sensor_values[NUM_SENSORS];
        uint16_t _min_sensor_values[NUM_SENSORS];
};

class Ultrasonic {
    public:
        Ultrasonic(uint trigger_pin, uint echo_pin);
        uint32_t get_distance_mm();
    private:
        uint _trigger_pin;
        uint _echo_pin;
        void _pulse_trigger();
        
};

#endif // SENSORS_HPP