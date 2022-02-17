#include "gpioHandler.h"

bool GPIOHandler::Init()
{
    pinMode(PWM_PINLOW, INPUT);
    pinMode(PWM_PINHIGH, INPUT);
    return true;
}

PWM_RETURN GPIOHandler::GetPWM()
{
    // Initialize variables
    float ontime, offtime, period;
    uint8_t freq, duty;
    PWM_RETURN pwm_to_return;

    // Read lowside pin, decease timeout to 100ms
    ontime = pulseIn(PWM_PINLOW,HIGH, 100000);
    offtime = pulseIn(PWM_PINLOW,LOW, 100000);
    period = ontime+offtime;
    freq = 1000000.0/period;
    duty = (ontime/period)*100;
    if ((int)freq == 0)
    {
        pwm_to_return.duty_cycle_low = 0;
        pwm_to_return.freq_low = 0;
    }
    else
    {
        pwm_to_return.duty_cycle_low = duty;
        pwm_to_return.freq_low = freq;
    }

    // Read highside pin, decease timeout to 100ms
    ontime = pulseIn(PWM_PINHIGH,HIGH, 100000);
    offtime = pulseIn(PWM_PINHIGH,LOW, 100000);
    period = ontime+offtime;
    freq = 1000000.0/period;
    duty = (ontime/period)*100;
    if ((int)freq == 0)
    {
        pwm_to_return.duty_cycle_high = 0;
        pwm_to_return.freq_high = 0;
    }
    else
    {
        pwm_to_return.duty_cycle_high = duty;
        pwm_to_return.freq_high = freq;
    }
    return pwm_to_return;
}
