#include "agv_commands.h"

#include "bt_kmn/bluetooth.h"
#include "bt_kmn/commands.h"

#include <stdio.h>
#include <stdlib.h>

// External references
extern bool robot_enabled;
extern float kp;
extern float ki;
extern float kd;

// Internal function pointers for motor control
static void (*_set_motor_speed_left)(int16_t) = nullptr;
static void (*_set_motor_speed_right)(int16_t) = nullptr;
static void (*_calibrate_sensors)() = nullptr;

// Setup function
void AGV_Commands_Setup(void (*set_left)(int16_t), void (*set_right)(int16_t), void (*calibrate)()) {
    _set_motor_speed_left = set_left;
    _set_motor_speed_right = set_right;
    _calibrate_sensors = calibrate;
}

// START command implementation
void _cmd_start(const char *params) {
    robot_enabled = true;
    Bluetooth_SendMessage("Robot STARTED\n");
    printf("Robot STARTED\n");
}

// STOP command implementation
void _cmd_stop(const char *params) {
    robot_enabled = false;
    
    // Stop motors if callbacks are set
    if (_set_motor_speed_left && _set_motor_speed_right) {
        _set_motor_speed_left(0);
        _set_motor_speed_right(0);
    }
    
    Bluetooth_SendMessage("Robot STOPPED\n");
    printf("Robot STOPPED\n");
}

// CALIBRATE command implementation
void _cmd_calibrate(const char *params) {
    Bluetooth_SendMessage("Starting calibration... Move robot over line (black and white)\n");
    printf("Starting calibration...\n");
    
    if (_calibrate_sensors) {
        _calibrate_sensors();
        Bluetooth_SendMessage("Calibration COMPLETE\n");
        printf("Calibration complete\n");
    } else {
        Bluetooth_SendMessage("ERROR: Calibration callback not set\n");
        printf("ERROR: Calibration callback not set\n");
    }
}

// PID command implementation
// Format: PID <kp> <ki> <kd>
// Example: PID 0.2 0.0 0.01
void _cmd_pid(const char *params) {
    if (params == nullptr || params[0] == '\0') {
        // No parameters - send current values
        char msg[64];
        snprintf(msg, sizeof(msg), "PID: Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);
        Bluetooth_SendMessage(msg);
        printf("%s", msg);
        return;
    }
    
    // Parse parameters
    float new_kp, new_ki, new_kd;
    int parsed = sscanf(params, "%f %f %f", &new_kp, &new_ki, &new_kd);
    
    if (parsed == 3) {
        kp = new_kp;
        ki = new_ki;
        kd = new_kd;
        
        char msg[64];
        snprintf(msg, sizeof(msg), "PID updated: Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);
        Bluetooth_SendMessage(msg);
        printf("%s", msg);
    } else {
        Bluetooth_SendMessage("ERROR: PID requires 3 values (Kp Ki Kd)\n");
        printf("ERROR: Invalid PID parameters\n");
    }
}

// Register commands
BT_COMMAND(START, _cmd_start)
BT_COMMAND(STOP, _cmd_stop)
BT_COMMAND(CALIBRATE, _cmd_calibrate)
BT_COMMAND(PID, _cmd_pid)
