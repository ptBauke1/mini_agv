#include "sensors.hpp"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <stdio.h>

SensorArray::SensorArray(uint bit0, uint bit1, uint bit2, uint adc)
    : _bit_0_pin(bit0), _bit_1_pin(bit1), _bit_2_pin(bit2), _adc_pin(adc) {
    gpio_init(_bit_0_pin);
    gpio_set_dir(_bit_0_pin, GPIO_OUT);
    gpio_init(_bit_1_pin);
    gpio_set_dir(_bit_1_pin, GPIO_OUT);
    gpio_init(_bit_2_pin);
    gpio_set_dir(_bit_2_pin, GPIO_OUT);

    adc_init();
    adc_gpio_init(_adc_pin);
    adc_select_input(0); // Assuming ADC channel 0 is used
}

void SensorArray::read_sensors() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        _set_mux_channel(i);
        _sensor_array[i] = _read_adc();
    }
}

uint16_t SensorArray::get_sensor_value(uint8_t index) {
    if (index < NUM_SENSORS) {
        return _sensor_array[index];
    }
    return 0;
}

void SensorArray::_set_mux_channel(uint8_t channel) {
    gpio_put(_bit_0_pin, (channel & 0x01));
    gpio_put(_bit_1_pin, (channel >> 1) & 0x01);
    gpio_put(_bit_2_pin, (channel >> 2) & 0x01);
}

uint16_t SensorArray::_read_adc() {
    adc_select_input(0); // Ensure the correct ADC channel is selected
    return adc_read();
}

float SensorArray::calculate_error() {
    int32_t weighted_sum = 0;
    uint16_t total_value = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        // normalizes the readings between 0 and 1000
        uint16_t sensor_value = (get_sensor_value(i) - _min_sensor_values[i]) * 1000 /
                                (_max_sensor_values[i] - _min_sensor_values[i]);
        weighted_sum += sensor_value * (i * 1000); // Weight by position
        total_value += sensor_value;
    }

    if (total_value == 0) {
        return 0.0f; // Avoid division by zero
    }

    return static_cast<float>(weighted_sum / total_value) - SENSOR_OFFSET; // Center around zero
}

void SensorArray::calibrate() {
    // Initialize min and max arrays
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        _max_sensor_values[i] = 0;
        _min_sensor_values[i] = 4095; // Max possible ADC value
    }

    // Perform calibration readings
    for (uint16_t sample = 0; sample < 20000; sample++) {
        read_sensors();
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            uint16_t value = get_sensor_value(i);
            if (value > _max_sensor_values[i]) {
                _max_sensor_values[i] = value;
            }
            if (value < _min_sensor_values[i]) {
                _min_sensor_values[i] = value;
            }
        }
    }
}

Ultrasonic::Ultrasonic(uint trigger_pin, uint echo_pin)
    : _trigger_pin(trigger_pin), _echo_pin(echo_pin) {
    gpio_init(_trigger_pin);
    gpio_set_dir(_trigger_pin, GPIO_OUT);
    gpio_init(_echo_pin);
    gpio_set_dir(_echo_pin, GPIO_IN);

    gpio_put(_trigger_pin, 0);
    sleep_ms(10);
}

void Ultrasonic::_pulse_trigger() {
    gpio_put(_trigger_pin, 1);
    sleep_us(10);
    gpio_put(_trigger_pin, 0);
}

uint32_t Ultrasonic::get_distance_mm() {
    _pulse_trigger();
    
    // Wait for echo to go high with timeout (38ms max for ~4m range)
    uint32_t timeout_start = time_us_32();
    while (gpio_get(_echo_pin) == 0) {
        if (time_us_32() - timeout_start > 38000) {
            return 0; // Timeout - no echo detected
        }
    }
    int32_t start_time = time_us_32();

    // Wait for echo to go low with timeout
    timeout_start = time_us_32();
    while (gpio_get(_echo_pin) == 1) {
        if (time_us_32() - timeout_start > 38000) {
            return 0; // Timeout - echo stuck high
        }
    }
    int32_t end_time = time_us_32();

    int32_t pulse_duration = end_time - start_time; // in microseconds
    // Calculate distance in mm (speed of sound is ~343 m/s = 0.343 mm/us)
    // Distance = (time * speed) / 2 = (pulse_duration_us * 0.343 mm/us) / 2
    uint32_t distance_mm = (pulse_duration * 343) / 2000;

    return distance_mm;
}