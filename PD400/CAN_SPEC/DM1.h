//PD400 CAN Spec 2.4 ENUMs to store the values for Faults
//Brennan

// Not used by the inverter, should be set to 0. used to indicate an emissions related fault.
enum MIL_e{         // Multi Indicator Lamp Status
    LAMPOFF,        // 0 = Lamp Off
    LAMPON,         // 1 = Lamp On
    RESERVED1,      // 2 = Reserved Not Used
    RESERVED2       // 3 = Reserved Not Used
};                  // NoFaultVal = 0; Min = 0; Max = 3

// Not used by the inverter, should be set to 0. Only useful if MIL = LAMPON
enum FMIL_e{        // Flash Multi Indicator Lamp Status
    LAMPOFF,        // 0 = Lamp Slow Flash
    LAMPON,         // 1 = Lamp Fast Flash
    RESERVED1,      // 2 = Reserved Not Used
    LAMPONNOFLASH   // 3 = Lamp On Not Flashing
};                  // NoFaultVal = 0; Min = 0; Max = 3

// Used to indicate that a Critical Fault has occurred.
enum RSL_e{         // Red Stop Lamp Status
    LAMPOFF,        // 0 = Lamp Off
    LAMPON,         // 1 = Lamp On
    RESERVED1,      // 2 = Reserved Not Used
    RESERVED2       // 3 = Reserved Not Used
};                  // NoFaultVal = 0; Min = 0; Max = 3

// Only useful if RSL = LAMPON
enum FRSL_e{        // Flash Red Stop Lamp Status
    LAMPOFF,        // 0 = Lamp Slow Flash
    LAMPON,         // 1 = Lamp Fast Flash
    RESERVED1,      // 2 = Reserved Not Used
    LAMPONNOFLASH   // 3 = Lamp On Not Flashing
};                  // NoFaultVal = 0; Min = 0; Max = 3

// Used to indicate that a Non-Critical Fault has occurred.
enum AWL_e{         // Amber Warning Lamp Status
    LAMPOFF,        // 0 = Lamp Off
    LAMPON,         // 1 = Lamp On
    RESERVED1,      // 2 = Reserved Not Used
    RESERVED2       // 3 = Reserved Not Used
};                  // NoFaultVal = 0; Min = 0; Max = 3

// Only useful if AWL = LAMPON
enum FAWL_e{        // Flash Red Stop Lamp Status
    LAMPOFF,        // 0 = Lamp Slow Flash
    LAMPON,         // 1 = Lamp Fast Flash
    RESERVED1,      // 2 = Reserved Not Used
    LAMPONNOFLASH   // 3 = Lamp On Not Flashing
};                  // NoFaultVal = 0; Min = 0; Max = 3

// Not used by the inverter, should be set to 0.
enum PL_e{          // Protect Lamp Status
    LAMPOFF,        // 0 = Lamp Off
    LAMPON,         // 1 = Lamp On
    RESERVED1,      // 2 = Reserved Not Used
    RESERVED2       // 3 = Reserved Not Used
};                  // NoFaultVal = 0; Min = 0; Max = 3

// Not used by the inverter, should be set to 0. Only useful if MIL = LAMPON
enum FPL_e{         // Flash Protect Lamp Status
    LAMPOFF,        // 0 = Lamp Slow Flash
    LAMPON,         // 1 = Lamp Fast Flash
    RESERVED1,      // 2 = Reserved Not Used
    LAMPONNOFLASH   // 3 = Lamp On Not Flashing
};                  // NoFaultVal = 0; Min = 0; Max = 3

// FMI(Fault Mode Indicator) Values      // Specific Use in Inverter description

#define VALID_OVER_RANGE_SEV     0x00   // 00, SEVERE, Covers Faults that exceed critical thresholds such as overcurrents.
#define VALID_BELOW_RANGE_SEV    0x01   // 01, SEVERE, Typically an under-volt event on high-voltage supply or low-volt batt connection that will affect inverter operations
#define ERRATIC_INTERMIT         0x02   // 02, Usually related to position feedback device or a communication issue.
#define VOLT_OVER_NORM           0x03   // 03, Related to critical power supply voltages going too high
#define VOLT_BELOW_NORM          0x04   // 04, Related to critical power supply voltages going too low
#define CURRENT_OVER_NORM        0x05   // 05, Phase missing faults
#define CURRENT_BELOW_NORM       0x06   // 06, Brake chopper failing to turn off
#define MECH_SYS_NO_RESP         0x07   // 07, Typically only seen when phase cable connection is lost or inverter running w/o motor attached
#define ABNORM_FREQ              0x08   // 08, Related to circuites that measure pulse widths, such as temp sens, or produce PWM signals
#define ABNORM_UPDATE_RATE       0x09   // 09, Related to something not executing within a given time period.
#define ABNORM_RATE_OF_CHANGE    0x0A   // 10, Related to something not executing within a given time period.
#define CAUSE_UNKNOWN            0x0B   // 11, Calibration failure for undetermined reason
#define BAD_COMPONENT            0x0C   // 12, Watchdog failure
#define OUT_OF_CALIB             0x0D   // 13, Usually inverter detects internal calibration is missing or inconsistent, or EOL or EEPROM checksum is incorrect.
#define SPECIAL_INSTRUCT         0x0E   // 14, Position not being calibrated
#define VALID_OVER_RANGE_LEAST   0x0F   // 15, Motor and coolant temp warnings
#define VALID_OVER_RANGE_MODER   0x10   // 16, Overvoltage and over-speed events
#define VALID_BELOW_RANGE_LEAST  0x11   // 17, Undervoltage events
#define VALID_BELOW_RANGE_MODER  0x12   // 18, Undervoltage lockout
#define CONDITION_EXISTS         0x1F   // 31, For fault that exists but have no other relevant characteristics