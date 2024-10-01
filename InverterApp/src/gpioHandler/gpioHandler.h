#include <stdlib.h>
#include "../Time/TimeLib.h"
#include <Arduino.h>
#include "../TaskScheduler/TaskScheduler.h"

#define POT_GPIO           A1
#define CLEAR_FAULT_GPIO   A5

#define SPEED_OFFSET   16000
#define SPEED_MIN_RPM      0
#define SPEED_MAX_RPM      600

#define MIN_SPEED_VAL (SPEED_OFFSET + SPEED_MIN_RPM) * 2
#define MAX_SPEED_VAL (SPEED_OFFSET + SPEED_MAX_RPM) * 2

// 0% -> 0.7%
#define TORQUE_OFFSET      16000
#define TORQUE_PERC_MIN    0
#define TORQUE_PERC_MAX    15

#define MIN_TORQUE_VAL (TORQUE_OFFSET + TORQUE_PERC_MIN) * 2
#define MAX_TORQUE_VAL (TORQUE_OFFSET + TORQUE_PERC_MAX) * 2

#define CLEAR_INTERVAL_MILLIS 5000

//If displays connected, uncomment next line.
#define DISPLAYS_ENABLED

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

#include "gpioHandlerMin.h"