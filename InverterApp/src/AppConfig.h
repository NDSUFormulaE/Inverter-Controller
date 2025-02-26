#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// ---------------------------- POWER MODE CONFIG -------------------------------

// Enable this value when we want to change the configuration to:
// - Send torque commands to the inverter instead of sending speed commands.
// - Use the accellerator pedals instead of a rotary potentiometer. 
#define USE_APPS

// ---------------------------- CAN CONFIG --------------------------------------
// Increase this if we actually need more than 1 task.
enum {MAX_CAN_TASKS = 1};

#define INVERTER_CMD_MESSAGE_INDEX 0
#define INVERTER_CMD_INTERVAL_MS 15
#define INVERTER_CMD_INVERVAL_TICKS pdMS_TO_TICKS(INVERTER_CMD_INTERVAL_MS)
#define CAN_CONTROL_LOOP_INTERVAL_MS 8
#define CAN_CONTROL_LOOP_INTERVAL_TICKS pdMS_TO_TICKS(CAN_CONTROL_LOOP_INTERVAL_MS)

#if INVERTER_CMD_INTERVAL_MS < CAN_CONTROL_LOOP_INTERVAL_MS
    #error "INVERTER_CMD_INTERVAL_MS must be greater than CAN_CONTROL_LOOP_INTERVAL_MS"
#endif

// Enable this value to have TaskClearFaults clear the faults when the
// inverter goes into either of the Fault Class states.
// #define AUTO_CLEAR_CAN_FAULTS

// ---------------------------- GPIO CONFIG -------------------------------------

#define POT_GPIO           A1
#define LEFT_APPS_GPIO     A2
#define RIGHT_APPS_GPIO    A3

#define CLEAR_FAULT_GPIO   A5

//minimum voltage 0.206V
#define LEFT_APPS_MIN_ADC 63
//Maximum voltage 1.157V
#define LEFT_APPS_MAX_ADC 358

//minimum voltage 0.492V
#define RIGHT_APPS_MIN_ADC 152
//Maximum voltage 1.496V
#define RIGHT_APPS_MAX_ADC 494

#define SPEED_OFFSET   16000
#define SPEED_MIN_RPM      0
#define SPEED_MAX_RPM      600

#define MIN_SPEED_VAL (SPEED_OFFSET + SPEED_MIN_RPM) * 2
#define MAX_SPEED_VAL (SPEED_OFFSET + SPEED_MAX_RPM) * 2

// 0% -> 0.7%
#define TORQUE_OFFSET      16000
#define TORQUE_PERC_MIN    0
#define TORQUE_PERC_MAX    5

#define MIN_TORQUE_VAL (TORQUE_OFFSET + TORQUE_PERC_MIN) * 2
#define MAX_TORQUE_VAL (TORQUE_OFFSET + TORQUE_PERC_MAX) * 2

#define CLEAR_INTERVAL_MILLIS 5000
#define CLEAR_INTERVAL_TICKS pdMS_TO_TICKS(CLEAR_INTERVAL_MILLIS)

//If displays connected, uncomment next line.
// #define DISPLAYS_ENABLED

// Speed Display Pins
#define SPD_CLK 22
#define SPD_DATA 24

// Battery Voltage Pins
#define BATT_CLK 26
#define BATT_DATA 28

// Motor Temperature Pins
#define TEMP_CLK 30
#define TEMP_DATA 32

// Coolant Temperature Pins
#define COOL_CLK 34
#define COOL_DATA 36

#endif /* APP_CONFIG_H */