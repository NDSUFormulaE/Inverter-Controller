//PD400 CAN Spec 2.3.6 Derate Owner
//Brennan

#define DERATE_NONE                  0x00  //00, No Derate
#define DERATE_IGBT_TEMP_LIMIT       0x01  //01, IGBT (Base Plate) Temperature Limit
#define DERATE_WINDING_TEMP_LIMIT    0x02  //02, Winding Temperature Limit
#define DERATE_SPEED_LIMIT           0x04  //04, Speed Limiting
#define DERATE_VOLT_LIMIT            0x05  //05, Voltage Limiting
#define DERATE_I2T_FAST_LIMIT        0x06  //06, Overload Current Protection (I2T) Fast Limit
#define DERATE_DIRECT_TORQ_LIMIT     0x0B  //11, Direct Torque Limit (CAN Command)
#define DERATE_TERMINAL_VOLTAGE      0x0C  //12, Terminal Voltage
#define DERATE_IGBT_JUNCT_TEMP       0x0D  //13, IGBT Junction Temperature
#define DERATE_I2T_SLOW_LIMIT        0x0C  //14, Overload Current Protection (I2T) Slow Limit
#define DERATE_PEAK_TORQ_CURVE       0x10  //16, Peak Torque Curve
#define DERATE_USER_DEF_TORQ_CURVE   0x11  //17, User Defined Torque Curve
#define DERATE_MSTR_SPEED_TORQ_CURVE 0x12  //18, Master Speed Torque Curve
#define DERATE_CURR_LIMIT_OVRLD      0x14  //20, Current Limiting (includes Inverter Current Overload)
#define DERATE_CURR_LIMIT_AC_SUPPLY  0x16  //22, Current Limiting(AC Supply)

#define NUM_DERATE_OWNERS            15