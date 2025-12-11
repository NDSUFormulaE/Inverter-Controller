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

TickType_t last_clear = 0;
uint8_t lcd_test = 0;

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

  pinMode(50, OUTPUT);
  tone(50, 1000);
    
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
  speedDisplay.showNumber(int(InverterState.Abs_Machine_Speed));
  batteryDisplay.showNumber(InverterState.DC_Bus_Voltage);
  motorTempDisplay.showNumber(int((InverterState.Motor_Temp_1 + InverterState.Motor_Temp_2 + InverterState.Motor_Temp_3)/3));
  avgTorqueDisplay.showNumber(int(InverterState.Avg_Abs_Torque));
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
  lcd.clear();
  lcd.print("Codes 0x"); lcd.print(lcd_test, HEX);
  for (int i=1; i<4; i++) {
    lcd.setCursor(0, i);
    for (int j=0; j<16; j++) {
      lcd.printByte(lcd_test+j);
    }
  }
  lcd_test+=16;
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
    if(analogReadVal > 500 && ((current_ticks - last_clear) > CLEAR_INTERVAL_TICKS))
    {
        last_clear = current_ticks;
        return analogReadVal;
    }
    return 0;
}