#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// ============================================================================
// FEATURE FLAGS
// ============================================================================

// Use APPS (accelerator pedals) for torque control instead of potentiometer for speed
#define USE_APPS

// Auto-clear faults when inverter enters fault states
// #define AUTO_CLEAR_CAN_FAULTS

// Enable serial debug output (115200 baud)
// #define DEBUG_SERIAL

// ============================================================================
// PERIPHERAL ENABLES
// ============================================================================

// If seven segment displays connected, uncomment next line
#define SEVEN_SEGMENT_DISPLAYS_ENABLED

// If LCD displays connected, uncomment next line
#define LCD_DISPLAY_ENABLED

// ============================================================================
// TASK TIMING (all values in milliseconds, use powers of 2)
// ============================================================================

// CAN bus polling interval
#define CAN_CONTROL_LOOP_INTERVAL_MS        8
#define CAN_CONTROL_LOOP_INTERVAL_TICKS     pdMS_TO_TICKS(CAN_CONTROL_LOOP_INTERVAL_MS)

// Inverter command message interval
#define INVERTER_CMD_INTERVAL_MS            32
#define INVERTER_CMD_INTERVAL_TICKS         pdMS_TO_TICKS(INVERTER_CMD_INTERVAL_MS)

// LCD display update interval
#define LCD_UPDATE_INTERVAL_MS              1024
#define LCD_UPDATE_INTERVAL_TICKS           pdMS_TO_TICKS(LCD_UPDATE_INTERVAL_MS)

// Seven segment display update interval
#define SEVEN_SEG_UPDATE_INTERVAL_MS        256
#define SEVEN_SEG_UPDATE_INTERVAL_TICKS     pdMS_TO_TICKS(SEVEN_SEG_UPDATE_INTERVAL_MS)

// Inverter state machine control interval
#define STATE_MACHINE_INTERVAL_MS           256
#define STATE_MACHINE_INTERVAL_TICKS        pdMS_TO_TICKS(STATE_MACHINE_INTERVAL_MS)

// Fault clear task interval
#define CLEAR_FAULTS_INTERVAL_MS            2048
#define CLEAR_FAULTS_INTERVAL_TICKS         pdMS_TO_TICKS(CLEAR_FAULTS_INTERVAL_MS)

// Message timeout (no response from device)
#define MSG_TIMEOUT_MS                      250
#define MSG_TIMEOUT_TICKS                   pdMS_TO_TICKS(MSG_TIMEOUT_MS)

#if INVERTER_CMD_INTERVAL_MS < CAN_CONTROL_LOOP_INTERVAL_MS
    #error "INVERTER_CMD_INTERVAL_MS must be greater than CAN_CONTROL_LOOP_INTERVAL_MS"
#endif

// ============================================================================
// CAN BUS CONFIG
// ============================================================================

// Max number of scheduled CAN tasks
enum {MAX_CAN_TASKS = 1};

// CAN Source Addresses
#define INVERTER_SOURCE_ADDRESS             0xA2
#define BMS_SOURCE_ADDRESS                  0x00  // TODO: Set to actual BMS source address

// Inverter command message index in CANTasks array
#define INVERTER_CMD_MESSAGE_INDEX          0

// ============================================================================
// GPIO PIN ASSIGNMENTS
// ============================================================================

// Analog Inputs
#define POT_GPIO                            A3
#define LEFT_APPS_GPIO                      A2
#define RIGHT_APPS_GPIO                     A1
#define CLEAR_FAULT_GPIO                    A5

// Seven Segment Display Pins (directly on board w/ 2-pin shift registers) - Speed Display
#define SPD_CLK                             22
#define SPD_DATA                            24

// Battery Voltage Display
#define BATT_CLK                            26
#define BATT_DATA                           28

// Motor Temperature Display
#define TEMP_CLK                            30
#define TEMP_DATA                           32

// Torque Display
#define TORQUE_CLK                          34
#define TORQUE_DATA                         36

// ============================================================================
// APPS (Accelerator Pedal Position Sensor) CALIBRATION
// ============================================================================

// Left APPS ADC range (0.206V min, 1.157V max)
#define LEFT_APPS_MIN_ADC                   70
#define LEFT_APPS_MAX_ADC                   355

// Right APPS ADC range (0.492V min, 1.496V max)
#define RIGHT_APPS_MIN_ADC                  170
#define RIGHT_APPS_MAX_ADC                  458

// ============================================================================
// SPEED COMMAND CONFIG (when USE_APPS is NOT defined)
// ============================================================================

#define SPEED_OFFSET                        16000
#define SPEED_GAIN                          0.5
#define SPEED_MIN_RPM                       0
#define SPEED_MAX_RPM                       600

#define MIN_SPEED_VAL                       (SPEED_OFFSET + SPEED_MIN_RPM) / SPEED_GAIN
#define MAX_SPEED_VAL                       (SPEED_OFFSET + SPEED_MAX_RPM) / SPEED_GAIN

// ============================================================================
// TORQUE COMMAND CONFIG (when USE_APPS IS defined)
// ============================================================================

#define TORQUE_OFFSET                       125
#define TORQUE_GAIN                         0.00390625
#define TORQUE_PERC_MIN                     0
// Max torque for bench testing - increase for real applications
#define TORQUE_PERC_MAX                     0.35

#define MIN_TORQUE_VAL                      (TORQUE_OFFSET + TORQUE_PERC_MIN) / TORQUE_GAIN
#define MAX_TORQUE_VAL                      (TORQUE_OFFSET + TORQUE_PERC_MAX) / TORQUE_GAIN

#endif /* APP_CONFIG_H */