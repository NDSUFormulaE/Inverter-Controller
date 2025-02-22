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
TickType_t last_clear = 0;
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
  //Initialized screen and backlight.
  lcd.init();
  lcd.backlight();
}

void GPIOHandler::LcdUpdate()
{
  lcd.clear();
  lcd.print("Codes 0x"); lcd.print(lcd_test, HEX);
  for (int i=1; i<4; i++) {
    lcd.setCursor(0, i);
    for (int j=0; j<16; j++) {
      lcd.printByte(lcd_test+j);
    }
  }
  lcd_test+=16;
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
  if(diff > 10){
    // Serial.print("Difference between left/right APPS is "); Serial.print(diff); Serial.println("%. Reading invalid, requesting 0\% instead.");
    return 0;
  }
  long apps_avg = (mapped_left_val + mapped_right_val) / 2;
  //Serial.print("Average Left/Right APPS Torque: "); Serial.print(apps_avg); Serial.println("%");
  uint16_t mapped_val = mapToRange(apps_avg, 0, 100, MIN_TORQUE_VAL, MAX_TORQUE_VAL);
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