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
    //speedDisplay.showString("NDSU SAE");
        //LCDDisplaySAE();
    #endif
    
    return true;
}
unsigned long last_clear = 0;

double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

void GPIOHandler::UpdateDisplays()
{
    speedDisplay.showNumber(int(InverterState.Abs_Machine_Speed));
    batteryDisplay.showNumber(InverterState.DC_Bus_Voltage);
    motorTempDisplay.showNumber(int((InverterState.Motor_Temp_1 + InverterState.Motor_Temp_2 + InverterState.Motor_Temp_3)/3));
    coolantTempDisplay.showNumber(InverterState.Inverter_Coolant_Temp);
    // LcdUpdate();
}

bool GPIOHandler::LcdInit()
{
  //Initialized screen and backlight.
  lcd.init();
  lcd.clear();
  lcd.backlight();
}

void GPIOHandler::LcdUpdateState(char* str_to_print)
{
  lcd.setCursor(0,0);
  // lcd.print("STATE:Norm Operation");
  lcd.print(str_to_print);
}
void GPIOHandler::LcdUpdateError1(char* str_to_print)
{
  lcd.setCursor(1,0);
  lcd.print(str_to_print);
}
void GPIOHandler::LcdUpdateError2(char* str_to_print)
{
  lcd.setCursor(2,0);
  lcd.print(str_to_print);
}
void GPIOHandler::LcdUpdateError3(char* str_to_print)
{
  lcd.setCursor(3,0);
  lcd.print(str_to_print);
}

void GPIOHandler::LCDDisplaySAE()
{
  //Characters used to create the logo
  lcd.createChar(0, fullBox);
  lcd.createChar(1, halfBoxBottom);
  lcd.createChar(2, halfBoxTop);
  lcd.createChar(3, topLeftAChar);
  lcd.createChar(4, sideLeftAChar);
  lcd.createChar(5, sideLeftAInnerChar);
  lcd.createChar(6, sideLeftInnerCharConnector);
  lcd.createChar(7, topRightSCurve);

  delay(3000);
  uint8_t i = 0;
  while (1) {
    lcd.clear();
    lcd.print("Codes 0x"); lcd.print(i, HEX);
    lcd.print("-0x"); lcd.print(i+16, HEX);
    lcd.setCursor(0, 1);
    for (int j=0; j<16; j++) {
      lcd.printByte(i+j);
    }
    i+=16;
    
    delay(4000);
  }
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
  int potent_read = analogRead(POT_GPIO);
  uint16_t mapped_val = map(potent_read, 0, 1023, MIN_TORQUE_VAL, MAX_TORQUE_VAL);
  return mapped_val;
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