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
    speedDisplay.showNumber(InverterState.Abs_Machine_Speed);
    batteryDisplay.showNumber(InverterState.DC_Bus_Voltage);
    motorTempDisplay.showNumber(InverterState.Motor_Temp_1);
    coolantTempDisplay.showNumber(InverterState.Inverter_Coolant_Temp);
}

bool GPIOHandler::LcdInit()
{
  //Initialized screen and backlight.
  lcd.init();
  lcd.backlight();
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

uint16_t GPIOHandler::GetPedalSpeed()
{
    int potent_read = analogRead(POT_GPIO);
    return map(potent_read, 0, 1023, SPEED_OFFSET + SPEED_MIN_RPM, SPEED_OFFSET + SPEED_MAX_RPM);;
}

uint16_t GPIOHandler::GetPedalTorque()
{
    int potent_read = analogRead(POT_GPIO);
    return map(potent_read, 0, 1023, TORQUE_OFFSET + TORQUE_PERC_MIN, TORQUE_OFFSET + TORQUE_PERC_MAX);;
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