#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "sensors.hpp"
#include "motors.hpp"
#include "gyro.hpp"
#include "agv_commands.h"

#include "bt_kmn/bluetooth.h"

SensorArray sensors(18, 19, 20, 26); // Sensores OK
MotorL298N left_motor(11, 14, 15);
MotorL298N right_motor(10, 12, 13);

Ultrasonic ultrasonic(2, 3); // Ultrassom OK

BluetoothConfig bt_config = {
    .uart = uart1,
    .baudrate = 9600,
    .rx_pin = 9,
    .tx_pin = 8
};

uint16_t aux;

int16_t base_speed = 800;
int16_t correction = 0;

float kp = 0.15f;
float ki = 0.0f;
float kd = 0.01f;
float last_error = 0.0f;
float integral = 0.0f;
bool object_detected = false;

uint32_t timer = 0;
#define TIME_STEP_US 1000 // 1 ms

uint32_t last_distance = 0;
int16_t left_pwm = 0;
int16_t right_pwm = 0;
float line_error = 0.0f;

// Robot state control
bool robot_enabled = false;  // Robot starts disabled

// Timer interrupt callback for Bluetooth telemetry
bool timer_callback(struct repeating_timer *t) {
    char msg[64];
    // Format: "L:<left_pwm>,R:<right_pwm>,E:<error>,D:<distance>"
    snprintf(msg, sizeof(msg), "L:%d,R:%d,E:%.2f,D:%lu\n", 
             left_pwm, right_pwm, line_error, last_distance);
    Bluetooth_SendMessage(msg);
    return true; // Keep repeating
}

// Wrapper functions for command system
void set_left_motor_speed(int16_t speed) {
    left_motor.set_speed(speed);
}

void set_right_motor_speed(int16_t speed) {
    right_motor.set_speed(speed);
}

void calibrate_sensors() {
    sensors.calibrate();
}

int main()
{
    stdio_init_all();

    Bluetooth_Setup(bt_config);
    sleep_ms(2000); // Wait for serial to initialize

    // Initialize command system
    AGV_Commands_Setup(set_left_motor_speed, set_right_motor_speed, calibrate_sensors);

    sensors.calibrate();
    sleep_ms(1000);

    struct repeating_timer timer_irq_bt;
    add_repeating_timer_ms(-300, timer_callback, NULL, &timer_irq_bt);

    printf("System ready. Waiting for START command...\n");

    while (true) {
        // Check for incoming commands via Bluetooth
        Bluetooth_ReadMessage();
        if (time_us_32() - timer > TIME_STEP_US) {
            sensors.read_sensors();
            last_distance = ultrasonic.get_distance_mm();
            float error = sensors.calculate_error();
            sensors.print_values();

            correction = kp * error + kd * (error - last_error) + ki * integral;
            last_error = error;
            integral += error;
            int16_t left_speed = base_speed + correction;
            int16_t right_speed = base_speed - correction;
    
            // Update global variables for Bluetooth transmission
            left_pwm = left_speed;
            right_pwm = right_speed;
            line_error = error;
    
            // Check if robot is enabled and no obstacle
            if (robot_enabled) {
                if (last_distance > 0 && last_distance < 120) { // Obstacle detected
                    left_speed = 0;
                    right_speed = 0;
                    object_detected = true;
                } else {
                    object_detected = false;
                }
                
                left_motor.set_speed(left_speed);
                right_motor.set_speed(right_speed);
            } else {
                // Robot disabled - ensure motors are off
                left_motor.set_speed(0);
                right_motor.set_speed(0);
                left_pwm = 0;
                right_pwm = 0;
            }
            
            timer = time_us_32();
        }

    }
}