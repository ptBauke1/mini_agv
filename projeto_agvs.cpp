#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"

#include "sensors.hpp"
#include "motors.hpp"
#include "gyro.hpp"

#include "bt_kmn/bluetooth.h"

SensorArray sensors(18, 19, 20, 26); // Sensores OK
Motor left_motor(14, 15); // motores OK
Motor right_motor(12, 13); // motores OK
Ultrasonic ultrasonic(21, 22); // Ultrassom OK

BluetoothConfig bt_config = {
    .uart = uart1,
    .baudrate = 9600,
    .rx_pin = 9,
    .tx_pin = 8
};

uint16_t aux;

int16_t base_speed = 700;
int16_t correction = 0;

float kp = 1.0f;
float ki = 0.0f;
float kd = 0.0f;
float last_error = 0.0f;
float integral = 0.0f;

uint32_t timer = 0;
#define TIME_STEP_US 1000 // 1 ms

uint32_t last_distance = 0;
int16_t left_pwm = 0;
int16_t right_pwm = 0;
float line_error = 0.0f;

// Timer interrupt callback for ultrasonic
bool ultrasonic_timer_callback(struct repeating_timer *t) {
    last_distance = ultrasonic.get_distance_mm();
    return true; // Keep repeating
}

// Timer interrupt callback for Bluetooth telemetry
bool timer_callback(struct repeating_timer *t) {
    char msg[64];
    // Format: "L:<left_pwm>,R:<right_pwm>,E:<error>,D:<distance>"
    snprintf(msg, sizeof(msg), "L:%d,R:%d,E:%.2f,D:%lu\n", 
             left_pwm, right_pwm, line_error, last_distance);
    Bluetooth_SendMessage(msg);
    return true; // Keep repeating
}

int main()
{
    stdio_init_all();

    Bluetooth_Setup(bt_config);
    sleep_ms(2000); // Wait for serial to initialize
    sensors.calibrate();
    sleep_ms(1000);

    struct repeating_timer timer_irq_bt;
    add_repeating_timer_ms(-100, timer_callback, NULL, &timer_irq_bt);
    struct repeating_timer timer_irq_ultrasonic;
    add_repeating_timer_ms(50, ultrasonic_timer_callback, NULL, &timer_irq_ultrasonic);

    while (true) {
        tight_loop_contents();
        if (time_us_32() - timer > TIME_STEP_US) {
            sensors.read_sensors();
            float error = sensors.calculate_error();
            
            correction = kp * error + kd * (error - last_error) + ki * integral;
            last_error = error;
            integral += error;

            int16_t left_speed = base_speed - correction;
            int16_t right_speed = base_speed + correction;
            
            // Update global variables for Bluetooth transmission
            left_pwm = left_speed;
            right_pwm = right_speed;
            line_error = error;
            
            //left_motor.set_speed(left_speed);
            //right_motor.set_speed(right_speed);
            
            timer = time_us_32();
        }
    }
}
