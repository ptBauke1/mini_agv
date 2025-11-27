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
        sleep_us(10); // Allow mux output to settle before ADC reading
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
    sleep_us(10);
}

uint16_t SensorArray::_read_adc() {
    adc_select_input(0); // Ensure the correct ADC channel is selected
    sleep_us(10); // Give ADC time to switch and settle
    return adc_read();
}

float SensorArray::calculate_error() {
    int32_t weighted_sum = 0;
    uint16_t total_value = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        // normalizes the readings between 0 and 1000
        uint16_t sensor_value = (get_sensor_value(i) - _min_sensor_values[i]) * 1000 /
                                (_max_sensor_values[i] - _min_sensor_values[i]);
        if (sensor_value > 1000) sensor_value = 1000;
        if (sensor_value < 0) sensor_value = 0;
        
        weighted_sum += sensor_value * (i * 1000); // Weight by position
        total_value += sensor_value;
    }

    if (total_value == 0) {
        return 0.0f; // Avoid division by zero
    }

    return static_cast<float>((weighted_sum / total_value) - SENSOR_OFFSET); // Center around zero
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

void SensorArray::print_values() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        printf("%d\t", get_sensor_value(i));
    }
    printf("\n");
}

Ultrasonic::Ultrasonic(uint trigger_pin, uint echo_pin)
    : _trigger_pin(trigger_pin), _echo_pin(echo_pin) {
    // Initialize trigger pin - force low multiple times
    gpio_init(_trigger_pin);
    gpio_set_dir(_trigger_pin, GPIO_OUT);
    gpio_put(_trigger_pin, 0);
    sleep_ms(10);
    gpio_put(_trigger_pin, 0); // Force low again
    
    // Initialize echo pin
    gpio_init(_echo_pin);
    gpio_set_dir(_echo_pin, GPIO_IN);
    gpio_set_pulls(_echo_pin, false, false); // Disable internal pulls for level shifter

    sleep_ms(100); // Longer initial delay for sensor startup
    gpio_put(_trigger_pin, 0); // Ensure trigger is still low
}
void Ultrasonic::_pulse_trigger() {
    gpio_put(_trigger_pin, 0);
    sleep_us(5); // Longer low time for level shifter
    gpio_put(_trigger_pin, 1);
    sleep_us(15); // Slightly longer pulse for reliability through level shifter
    gpio_put(_trigger_pin, 0);
    sleep_us(2); // Small settling time
}

uint32_t Ultrasonic::get_distance_mm() {
    // Ensure trigger starts low, then send pulse
    gpio_put(_trigger_pin, 0);
    sleep_us(2);
    gpio_put(_trigger_pin, 1);
    sleep_us(10);
    gpio_put(_trigger_pin, 0);

    // Wait for echo to go high with timeout
    uint32_t timeout_start = time_us_32();
    while(gpio_get(_echo_pin) == 0) {
        if (time_us_32() - timeout_start > 50000) {
            return 0; // Timeout waiting for echo start
        }
    }
    
    // Measure pulse width with timeout
    uint32_t start_time = time_us_32();
    while(gpio_get(_echo_pin) == 1) {
        if (time_us_32() - start_time > 50000) {
            return 0; // Timeout - pulse too long
        }
    }
    uint32_t pulse_width = time_us_32() - start_time;
    
    // Sanity check pulse duration (150us - 25ms is valid range)
    if (pulse_width < 150 || pulse_width > 25000) {
        return 0; // Invalid reading
    }
    
    // Calculate distance in mm (speed of sound is ~343 m/s = 0.343 mm/us)
    // Distance = (time * speed) / 2 = (pulse_width_us * 0.343 mm/us) / 2
    uint32_t distance_mm = (pulse_width * 343) / 2000;

    return distance_mm;
}