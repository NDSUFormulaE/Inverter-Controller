#include "gpioHandler.h"
#include "../TM1637TinyDisplay/TM1637TinyDisplay.h"

extern struct CANVariables InverterState;

uint16_t speed = 0;

TM1637TinyDisplay speedDisplay(SPD_CLK, SPD_DATA), batteryDisplay(BATT_CLK,BATT_DATA), motorTempDisplay(TEMP_CLK,TEMP_DATA), coolantTempDisplay(COOL_CLK,COOL_DATA);

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
        //speedDisplay.showString("NDSU SAE");
        speedDisplay.showNumber(43.21);
        batteryDisplay.showNumber(123.4);
        coolantTempDisplay.showNumber(43.21);
        motorTempDisplay.showNumber(000.0);
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