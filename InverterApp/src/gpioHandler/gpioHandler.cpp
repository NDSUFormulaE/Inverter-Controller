#include "gpioHandler.h"
#include "lcd.h" 
#include "../TM1637TinyDisplay/TM1637TinyDisplay.h"
#include "../LiquidCrystal_I2C/LiquidCrystal_I2C.h"

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

extern struct CANVariables InverterState;

uint16_t speed = 0;

TM1637TinyDisplay speedDisplay(SPD_CLK, SPD_DATA), batteryDisplay(BATT_CLK,BATT_DATA), motorTempDisplay(TEMP_CLK,TEMP_DATA), coolantTempDisplay(COOL_CLK,COOL_DATA);
LiquidCrystal_I2C lcd(0x27,20,4);

SemaphoreHandle_t lcdMutex;
char displayBuffer[4][20];
char newBuffer[4][20];
bool needsUpdate;

bool GPIOHandler::Init()
{
    // Configure all of our GPIOs
    #ifdef DISPLAYS_ENABLED
        // ALL OF OUR CODE
        Serial.println("Starting Displays");
        speedDisplay.begin();
        batteryDisplay.begin();
        motorTempDisplay.begin();
        coolantTempDisplay.begin();
        GPIOHandler::LcdInit();
    #endif
    
    return true;
}
unsigned long last_clear = 0;
uint8_t lcd_test = 0;

double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

void GPIOHandler::UpdateLCDs() {
  LcdUpdate();
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
    lcdMutex = xSemaphoreCreateMutex();
    if (lcdMutex == NULL) {
        return false;
    }
    
    //Initialize screen and backlight
    lcd.init();
    lcd.backlight();
    
    // Initialize buffers
    memset(displayBuffer, ' ', sizeof(displayBuffer));
    memset(newBuffer, ' ', sizeof(newBuffer));
    needsUpdate = false;
    
    return true;
}

void GPIOHandler::LcdUpdate()
{
    if (xSemaphoreTake(lcdMutex, portMAX_DELAY) == pdTRUE) {
        // Update first line with test pattern
        snprintf(newBuffer[0], 20, "Codes 0x%02X", lcd_test);
        
        // Update other lines
        for (int i = 1; i < 4; i++) {
            for (int j = 0; j < 16; j++) {
                newBuffer[i][j] = lcd_test + j;
            }
            newBuffer[i][16] = '\0';
        }
        
        // Only update changed lines
        for (int i = 0; i < 4; i++) {
            if (memcmp(displayBuffer[i], newBuffer[i], 20) != 0) {
                lcd.setCursor(0, i);
                lcd.print(newBuffer[i]);
                memcpy(displayBuffer[i], newBuffer[i], 20);
            }
        }
        
        lcd_test += 16;
        xSemaphoreGive(lcdMutex);
    }
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