#include "gpioHandler.h"
#include "lcd.h" 
#include "../TM1637TinyDisplay/TM1637TinyDisplay.h"
#include "../LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "../FreeRTOS/src/Arduino_FreeRTOS.h"
#include "../ARD1939/CAN_SPEC/MotorControlUnitState.h"
#include "../ARD1939/CAN_SPEC/CANVariables.h"

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

extern struct CANVariables InverterState;
extern struct FaultEntry FaultTable[MAX_FAULTS];
extern TaskScheduler taskMan;

TickType_t last_clear = 0;

#ifdef LCD_DISPLAY_ENABLED
// LCD buffer for dirty-checking (20 cols x 4 rows)
#define LCD_COLS 20
#define LCD_ROWS 4
static char lcdBuffer[LCD_ROWS][LCD_COLS + 1];      // Current display content
static char lcdNewBuffer[LCD_ROWS][LCD_COLS + 1];   // New content to display
static bool lcdInitialized = false;
static char lcdStatusLine[LCD_COLS + 1] = "";       // Line 4 status message buffer

// Convert MCU state to short string (max 8 chars for display)
static const char* getMcuStateStr(uint32_t state) {
    switch(state) {
        case MCU_PWR_UP:           return "PWRUP";
        case MCU_FUNCTIONAL_DIAG:  return "FDIAG";
        case MCU_FAULT_CLASSA:     return "FAULTA";
        case MCU_IGNIT_READY:      return "IGNRDY";
        case MCU_PWR_READY:        return "PWRRDY";
        case MCU_PWR_DIAG:         return "PDIAG";
        case MCU_DRIVE_READY:      return "DRVRDY";
        case MCU_NORM_OPS:         return "NORMOP";
        case MCU_FAULT_CLASSB:     return "FAULTB";
        case MCU_CNTRL_PWR_DOWN:   return "PWRDN";
        case MCU_FAIL_SAFE:        return "FAILSF";
        case MCU_ADV_DIAG_CLASSA:  return "ADIAGA";
        case MCU_DISCHARGE_DIAG:   return "DSCHG";
        case MCU_ADV_DIAG_CLASSB:  return "ADIAGB";
        default:                   return "UNK";
    }
}

// Write a line to the new buffer (pads/truncates to LCD_COLS)
static void lcdSetLine(uint8_t row, const char* text) {
    if (row >= LCD_ROWS) return;
    int i = 0;
    while (i < LCD_COLS && text[i] != '\0') {
        lcdNewBuffer[row][i] = text[i];
        i++;
    }
    while (i < LCD_COLS) {
        lcdNewBuffer[row][i] = ' ';
        i++;
    }
    lcdNewBuffer[row][LCD_COLS] = '\0';
}
#endif

// Set a status message to display on line 4 (replaces Serial.println for display)
void LcdPrintStatus(const char* msg) {
    #ifdef LCD_DISPLAY_ENABLED.
    Serial.println(msg); // Also print to serial for debugging
    strncpy(lcdStatusLine, msg, LCD_COLS);
    lcdStatusLine[LCD_COLS] = '\0';
    #endif
    Serial.println(msg); // Also print to serial for debugging
}

#ifdef SEVEN_SEGMENT_DISPLAYS_ENABLED
TM1637TinyDisplay speedDisplay(SPD_CLK, SPD_DATA), batteryDisplay(BATT_CLK,BATT_DATA), motorTempDisplay(TEMP_CLK,TEMP_DATA), avgTorqueDisplay(TORQUE_CLK,TORQUE_DATA);
#endif

#ifdef LCD_DISPLAY_ENABLED
LiquidCrystal_I2C lcd(0x27,20,4);
#endif

bool GPIOHandler::Init()
{
  #ifdef SEVEN_SEGMENT_DISPLAYS_ENABLED
    Serial.println("Starting Seven Segment Displays");
    speedDisplay.begin();
    batteryDisplay.begin();
    motorTempDisplay.begin();
    avgTorqueDisplay.begin();
  #endif

  #ifdef LCD_DISPLAY_ENABLED
    Serial.println("Starting LCD Display");
    LcdInit();
  #endif
    
  return true;
}

double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

void GPIOHandler::UpdateLCDs() {
  LcdUpdate();
}

void GPIOHandler::UpdateSevenSegments() {
  #ifdef SEVEN_SEGMENT_DISPLAYS_ENABLED
  // Check if inverter is disconnected (no messages for timeout period)
  uint32_t now = (uint32_t)xTaskGetTickCount();
  bool ivtrMissing = (InverterState.Last_Inverter_Msg_Time == 0 || 
                      (now - InverterState.Last_Inverter_Msg_Time) > MSG_TIMEOUT_TICKS);
  
  if (ivtrMissing) {
    // Blank all displays when inverter disconnected
    speedDisplay.clear();
    batteryDisplay.clear();
    motorTempDisplay.clear();
    avgTorqueDisplay.clear();
  } else {
    speedDisplay.showNumber(int(InverterState.Abs_Machine_Speed));
    batteryDisplay.showNumber(InverterState.DC_Bus_Voltage);
    motorTempDisplay.showNumber(int((InverterState.Motor_Temp_1 + InverterState.Motor_Temp_2 + InverterState.Motor_Temp_3)/3));
    avgTorqueDisplay.showNumber(int(InverterState.Avg_Abs_Torque));
  }
  #endif
}

bool GPIOHandler::LcdInit()
{
  //Initialized screen and backlight.
  #ifdef LCD_DISPLAY_ENABLED
  lcd.init();
  Wire.setClock(400000);
  lcd.backlight();
  #endif
}

void GPIOHandler::LcdUpdate()
{
  #ifdef LCD_DISPLAY_ENABLED
  char lineBuf[LCD_COLS + 1];
  
  // Initialize buffers on first run
  if (!lcdInitialized) {
    for (int r = 0; r < LCD_ROWS; r++) {
      memset(lcdBuffer[r], ' ', LCD_COLS);
      lcdBuffer[r][LCD_COLS] = '\0';
      memset(lcdNewBuffer[r], ' ', LCD_COLS);
      lcdNewBuffer[r][LCD_COLS] = '\0';
    }
    lcdInitialized = true;
  }
  
  // Line 1: System Status (CAN, BMS, IVTR)
  uint32_t now = (uint32_t)xTaskGetTickCount();
  
  // Determine CAN status string
  // EFLG bits: RX1OVR(7) RX0OVR(6) TXBO(5) TXEP(4) RXEP(3) TXWAR(2) RXWAR(1) EWARN(0)
  const char* canStatus;
  uint8_t eflg = InverterState.CAN_Hardware_Error;
  if (eflg & 0x20) {  // TXBO - bus off
    canStatus = "BOFF";
  } else if (eflg & 0x18) {  // TXEP(4) or RXEP(3) - error passive
    canStatus = "ERRP";
  } else if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FAILED) {
    canStatus = "FAIL";
  } else if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED && taskMan.GetSourceAddress() == 0xFE) {
    canStatus = "NULL";
  } else if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED) {
    canStatus = "OK  ";
  } else {
    canStatus = "--  ";
  }
  
  // Determine BMS status string (with timeout check)
  const char* bmsStatus;
  if (InverterState.Last_BMS_Msg_Time == 0 || (now - InverterState.Last_BMS_Msg_Time) > MSG_TIMEOUT_TICKS) {
    bmsStatus = "MIS";  // MISSING - no messages received
  } else if (InverterState.BMS_Status == BMS_STATUS_OK) {
    bmsStatus = "OK ";
  } else if (InverterState.BMS_Status == BMS_STATUS_FAULT) {
    bmsStatus = "FLT";
  } else {
    bmsStatus = "-- ";
  }
  
  // Determine Inverter status string (with timeout check)
  const char* ivtrStatus;
  bool ivtrMissing = (InverterState.Last_Inverter_Msg_Time == 0 || 
                      (now - InverterState.Last_Inverter_Msg_Time) > MSG_TIMEOUT_TICKS);
  if (ivtrMissing) {
    ivtrStatus = "MIS";  // MISSING - no messages received
  } else {
    ivtrStatus = "OK ";
  }
  
  // Line 1: CAN status + MCU State (show UNK if inverter missing)
  const char* mcuStateStr = ivtrMissing ? "UNK" : getMcuStateStr(InverterState.MCU_State);
  snprintf(lineBuf, sizeof(lineBuf), "CAN:%s MODE:%s", canStatus, mcuStateStr);
  lcdSetLine(0, lineBuf);
  
  // Line 2: BMS + Inverter comm status
  snprintf(lineBuf, sizeof(lineBuf), "IVTR:%s BMS:%s", ivtrStatus, bmsStatus);
  lcdSetLine(1, lineBuf);
  
  // Lines 3-4: Active faults (SPN:FMI format) - 2 faults per line, 2 lines
  char faultLine1[LCD_COLS + 1] = "";
  char faultLine2[LCD_COLS + 1] = "";
  int faultCount = 0;
  int pos1 = 0, pos2 = 0;
  
  for (int i = 0; i < MAX_FAULTS && faultCount < 4; i++) {
    if (FaultTable[i].active) {
      char faultStr[16];
      snprintf(faultStr, sizeof(faultStr), "%lu:%u#%u ", FaultTable[i].SPN, FaultTable[i].FMI, FaultTable[i].OC);
      int len = strlen(faultStr);
      
      if (faultCount < 2) {
        if (pos1 + len <= LCD_COLS) {
          strcat(faultLine1, faultStr);
          pos1 += len;
          faultCount++;
        }
      } else {
        if (pos2 + len <= LCD_COLS) {
          strcat(faultLine2, faultStr);
          pos2 += len;
          faultCount++;
        }
      }
    }
  }
  
  // Count total active faults for debug
  int totalActive = 0;
  for (int i = 0; i < MAX_FAULTS; i++) {
    if (FaultTable[i].active) totalActive++;
  }
  
  if (faultCount == 0) {
    char debugLine[LCD_COLS + 1];
    snprintf(debugLine, sizeof(debugLine), "Faults: %d active", totalActive);
    lcdSetLine(2, debugLine);
    lcdSetLine(3, "");
  } else {
    lcdSetLine(2, faultLine1);
    lcdSetLine(3, faultLine2);
  }
  
  // Update only changed characters (dirty-check)
  for (int row = 0; row < LCD_ROWS; row++) {
    bool rowChanged = false;
    for (int col = 0; col < LCD_COLS; col++) {
      if (lcdBuffer[row][col] != lcdNewBuffer[row][col]) {
        rowChanged = true;
        break;
      }
    }
    
    if (rowChanged) {
      lcd.setCursor(0, row);
      lcd.print(lcdNewBuffer[row]);
      memcpy(lcdBuffer[row], lcdNewBuffer[row], LCD_COLS + 1);
    }
    taskYIELD(); // Yield after each row check
  }
  #endif
}

uint16_t mapToRange(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (uint16_t)(((long)x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

uint16_t GPIOHandler::GetPedalSpeed()
{
  int potent_read = analogRead(POT_GPIO);
  uint16_t mapped_val = mapToRange(potent_read, 0, 1023, MIN_SPEED_VAL, MAX_SPEED_VAL);
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
  long mapped_left_val = map(left_apps_read, LEFT_APPS_MIN_ADC, LEFT_APPS_MAX_ADC, 0, 100);
  long mapped_right_val = map(right_apps_read, RIGHT_APPS_MIN_ADC, RIGHT_APPS_MAX_ADC, 0, 100);
  // Serial.print("Left: "); Serial.print(mapped_left_val); Serial.print(" Right: "); Serial.println(mapped_right_val);
  long diff = abs(mapped_right_val - mapped_left_val);
  long apps_avg = (mapped_left_val + mapped_right_val) / 2;
  // TODO: We need to improve this logic so that for any amount of failures here that falls under the safe amount in
  // the rules that we revert to some average of the last few values instead of setting it to 0.
  if(diff > 10){
    apps_avg = 0;
  }
  // Serial.print("Average Left/Right APPS Torque Mapped: ");
  uint16_t mapped_val = mapToRange(apps_avg, 0, 100, MIN_TORQUE_VAL, MAX_TORQUE_VAL);
  // Serial.println(mapped_val);
  return mapped_val;
}

uint16_t GPIOHandler::GetClearPin()
{
    uint16_t analogReadVal = analogRead(CLEAR_FAULT_GPIO);
    TickType_t current_ticks = xTaskGetTickCount();
    if(analogReadVal > 500 && ((current_ticks - last_clear) > CLEAR_FAULTS_INTERVAL_TICKS))
    {
        last_clear = current_ticks;
        return analogReadVal;
    }
    return 0;
}