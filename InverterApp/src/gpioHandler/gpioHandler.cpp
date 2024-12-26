#include "gpioHandler.h"
#include "lcd.h" 
#include "../TM1637TinyDisplay/TM1637TinyDisplay.h"
#include "../LiquidCrystal_I2C/LiquidCrystal_I2C.h"

extern struct CANVariables InverterState;

uint16_t speed = 0;

TM1637TinyDisplay speedDisplay(SPD_CLK, SPD_DATA), batteryDisplay(BATT_CLK,BATT_DATA), motorTempDisplay(TEMP_CLK,TEMP_DATA), coolantTempDisplay(COOL_CLK,COOL_DATA);
LiquidCrystal_I2C lcd(0x27,20,4);

SemaphoreHandle_t lcdMutex;
char displayBuffer[4][20];
char newBuffer[4][20];
bool needsUpdate;
bool persistInfoLines = false;

bool GPIOHandler::Init()
{
    speed = 0;
    needsUpdate = false;
    persistInfoLines = false;
    
    // Initialize display buffers
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 20; j++) {
            displayBuffer[i][j] = ' ';
            newBuffer[i][j] = ' ';
        }
    }
    
    lcdMutex = xSemaphoreCreateMutex();
    if (lcdMutex == NULL) {
        return false;
    }
    
    return LcdInit();
}

unsigned long last_clear = 0;
uint8_t lcd_test = 0;

double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

void GPIOHandler::UpdateLCDs() {
    if (xSemaphoreTake(lcdMutex, portMAX_DELAY) == pdTRUE) {
        // Only update changed lines
        for (int i = 0; i < 4; i++) {
            if (strncmp(displayBuffer[i], newBuffer[i], 20) != 0) {
                lcd.setCursor(0, i);
                lcd.print(newBuffer[i]);
                strncpy(displayBuffer[i], newBuffer[i], 20);
            }
        }
        xSemaphoreGive(lcdMutex);
    }
}

void GPIOHandler::UpdateSevenSegments() {
  #ifndef USE_APPS
  speedDisplay.showNumber(int(InverterState.Abs_Machine_Speed));
  #else
  speedDisplay.showNumber(int(InverterState.Avg_Abs_Torque));
  #endif
  batteryDisplay.showNumber(InverterState.DC_Bus_Voltage);
  motorTempDisplay.showNumber(int((InverterState.Motor_Temp_1 + InverterState.Motor_Temp_2 + InverterState.Motor_Temp_3)/3));
  coolantTempDisplay.showNumber(InverterState.Inverter_Coolant_Temp);
}

bool GPIOHandler::LcdInit()
{
    //Initialize screen and backlight
    lcd.init();
    lcd.backlight();
    
    return true;
}

void GPIOHandler::UpdateDisplayText(const char* text, uint8_t row, uint8_t col)
{
    if (row >= 4 || col >= 20) return;
    
    if (xSemaphoreTake(lcdMutex, 0) == pdTRUE) {  // Non-blocking semaphore take
        strncpy(&newBuffer[row][col], text, 20 - col);
        needsUpdate = true;
        xSemaphoreGive(lcdMutex);
    }
}

uint16_t mapToRange(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (uint16_t)(((long)x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

uint16_t GPIOHandler::GetPedalSpeed()
{
  int potent_read = analogRead(POT_GPIO);
  uint16_t mapped_val = mapToRange(potent_read, 0, 1023, MIN_SPEED_VAL, MAX_SPEED_VAL);
  Serial.println(potent_read);
  return mapped_val;
}

uint16_t GPIOHandler::GetPedalTorque()
{
  int left_apps_read = analogRead(LEFT_APPS_GPIO);
  if(left_apps_read > LEFT_APPS_MAX_ADC) {
    left_apps_read = LEFT_APPS_MAX_ADC;

  }
  if(left_apps_read < LEFT_APPS_MIN_ADC) {
    left_apps_read = LEFT_APPS_MIN_ADC;
  }
  int right_apps_read = analogRead(RIGHT_APPS_GPIO);
  if(right_apps_read > RIGHT_APPS_MAX_ADC) {
    right_apps_read = RIGHT_APPS_MAX_ADC;
  }
  if(right_apps_read < RIGHT_APPS_MIN_ADC) {
    right_apps_read = RIGHT_APPS_MIN_ADC;
  }
  uint16_t mapped_left_val = map(left_apps_read, LEFT_APPS_MIN_ADC, LEFT_APPS_MAX_ADC, 0, 100);
  // uint16_t mapped_right_val = map(right_apps_read, RIGHT_APPS_MIN_ADC, RIGHT_APPS_MAX_ADC, 0, 100);
  // int diff = abs(mapped_right_val - mapped_left_val);
  // if(diff > 10){
  //   return 0;
  // }
  // uint16_t apps_avg = (mapped_left_val + mapped_right_val) / 2;
  uint16_t mapped_val = mapToRange(mapped_left_val, 0, 100, MIN_TORQUE_VAL, MAX_TORQUE_VAL);
  return mapped_left_val;
}

uint16_t GPIOHandler::GetClearPin()
{
    uint16_t analogReadVal = analogRead(CLEAR_FAULT_GPIO);
    if(analogReadVal > 500 && ((millis() - last_clear) > CLEAR_INTERVAL_MILLIS))
    {
        last_clear = millis();
        return analogReadVal;
    }
    return 0;
}

const char* GPIOHandler::getStateName(uint8_t state) {
    switch(state) {
        case MCU_PWR_UP:  // Same as MCU_STDBY (0x00)
            return "Standby/Power Up";
        case MCU_FUNCTIONAL_DIAG: return "Functional Diag";
        case MCU_FAULT_CLASSA: return "Fault Class A";
        case MCU_IGNIT_READY: return "Ignition Ready";
        case MCU_PWR_READY: return "Power Ready";
        case MCU_PWR_DIAG: return "Power Diagnostics";
        case MCU_DRIVE_READY: return "Drive Ready";
        case MCU_NORM_OPS: return "Normal Operation";
        case MCU_FAULT_CLASSB: return "Fault Class B";
        case MCU_CNTRL_PWR_DOWN: return "Power Down";
        case MCU_FAIL_SAFE: return "Fail Safe";
        case MCU_ADV_DIAG_CLASSA: return "Adv Diag Class A";
        case MCU_DISCHARGE_DIAG: return "Discharge Diag";
        case MCU_ADV_DIAG_CLASSB: return "Adv Diag Class B";
        default: return "Unknown State";
    }
}

const char* GPIOHandler::getCANStatusName(uint8_t status) {
    switch(status) {
        case ADDRESSCLAIM_INIT: return "Initializing";
        case ADDRESSCLAIM_INPROGRESS: return "In Progress";
        case ADDRESSCLAIM_FINISHED: return "Connected";
        case ADDRESSCLAIM_FAILED: return "Failed";
        default: return "Unknown Status";
    }
}

void GPIOHandler::UpdateState(const char* state) {
    if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        UpdateDisplayText(state, 0, 0);
        if (!persistInfoLines) {
            // Clear info lines if not persisting
            UpdateDisplayText("", 2, 0);
            UpdateDisplayText("", 3, 0);
        }
        xSemaphoreGive(lcdMutex);
    }
}

void GPIOHandler::UpdateFault(const char* fault) {
    if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        UpdateDisplayText(fault, 1, 0);
        if (!persistInfoLines) {
            // Clear info lines if not persisting
            UpdateDisplayText("", 2, 0);
            UpdateDisplayText("", 3, 0);
        }
        xSemaphoreGive(lcdMutex);
    }
}

void GPIOHandler::UpdateInfo(const char* info1, const char* info2, bool persist) {
    if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        persistInfoLines = persist;
        UpdateDisplayText(info1, 2, 0);
        UpdateDisplayText(info2, 3, 0);
        xSemaphoreGive(lcdMutex);
    }
}

void GPIOHandler::UpdateLCD(const char* state, const char* fault, const char* info1, const char* info2) {
    if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        UpdateDisplayText(state, 0, 0);
        UpdateDisplayText(fault, 1, 0);
        UpdateDisplayText(info1, 2, 0);
        UpdateDisplayText(info2, 3, 0);
        xSemaphoreGive(lcdMutex);
    }
}

void GPIOHandler::DisplayStateTransition(uint8_t state, const char* transitionMsg, bool persistInfo) {
    const char* stateName = getStateName(state);
    UpdateState(stateName);
    UpdateFault("No Faults");
    UpdateInfo(transitionMsg, "", persistInfo);
}

void GPIOHandler::DisplayFaultState(uint8_t state, const char* faultType, const char* action, bool persistInfo) {
    char stateStr[20];
    snprintf(stateStr, sizeof(stateStr), "State: 0x%02X", state);
    UpdateState(stateStr);
    UpdateFault(faultType);
    UpdateInfo(action, "", persistInfo);
}