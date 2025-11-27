#pragma once

#include <stdint.h>

// Setup function to initialize AGV commands with motor control and calibration callbacks
// Parameters:
//   set_left: Function pointer to set left motor speed
//   set_right: Function pointer to set right motor speed
//   calibrate: Function pointer to calibrate line sensors
void AGV_Commands_Setup(void (*set_left)(int16_t), void (*set_right)(int16_t), void (*calibrate)());
