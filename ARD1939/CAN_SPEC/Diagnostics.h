//PD400 CAN Spec 2.3.5 Diagnostics Function
//Brennan

#define DIAG_OFF                                   0xFF  //255,  Not in a diagnostic state

// Advanced Diagnostics A
    #define DIAG_AD_A_DO_NOTHING                   0x64  //100, Advanced Diagnostics A - Do nothing
    #define DIAG_AD_A_CURR_SENS_ZERO_OFFSET_CALIB  0x66  //102, Advanced Diagnostics A - Current sensor zero offset calibration

    // Current Sensor
    #define DIAG_AD_A_PHASE_A_CURR_SENS_DIS        0x67  //103, Advanced Diagnostics A – Phase A current sensor disable
    #define DIAG_AD_A_PHASE_A_CURR_SENS_EN         0x68  //104, Advanced Diagnostics A - Phase A current sensor enable
    #define DIAG_AD_A_PHASE_B_CURR_SENS_DIS        0x69  //105, Advanced Diagnostics A – Phase B current sensor disable
    #define DIAG_AD_A_PHASE_B_CURR_SENS_EN         0x6A  //106, Advanced Diagnostics A - Phase B current sensor enable
    #define DIAG_AD_A_PHASE_C_CURR_SENS_DIS        0x6B  //107, Advanced Diagnostics A – Phase C current sensor disable
    #define DIAG_AD_A_PHASE_C_CURR_SENS_EN         0x6C  //108, Advanced Diagnostics A - Phase C current sensor enable

    #define DIAG_AD_A_BRAKE_RES_CURR_SENS_DIS      0x6D  //109, Advanced Diagnostics A – Brake resistor current sensor disable
    #define DIAG_AD_A_BRAKE_RES_CURR_SENS_EN       0x6E  //110, Advanced Diagnostics A - Brake resistor current sensor enable

    // Winding Temp
    #define DIAG_AD_A_WIND_TEMP_SENS_A_DIS         0x6F  //111, Advanced Diagnostics A - Winding temperature sensor A disable
    #define DIAG_AD_A_WIND_TEMP_SENS_A_EN          0x70  //112, Advanced Diagnostics A - Winding temperature sensor A enable
    #define DIAG_AD_A_WIND_TEMP_SENS_B_DIS         0x71  //113, Advanced Diagnostics A - Winding temperature sensor B disable
    #define DIAG_AD_A_WIND_TEMP_SENS_B_EN          0x72  //114, Advanced Diagnostics A - Winding temperature sensor B enable
    #define DIAG_AD_A_WIND_TEMP_SENS_C_DIS         0x73  //115, Advanced Diagnostics A - Winding temperature sensor C disable
    #define DIAG_AD_A_WIND_TEMP_SENS_C_EN          0x74  //116, Advanced Diagnostics A - Winding temperature sensor C enable

    // IGBT Temp Sensor
    #define DIAG_AD_A_IGBT_TEMP_SENS_A_DIS         0x76  //118, Advanced Diagnostics A - IGBT temperature sensor A disable
    #define DIAG_AD_A_IGBT_TEMP_SENS_A_EN          0x77  //119, Advanced Diagnostics A - IGBT temperature sensor A enable
    #define DIAG_AD_A_IGBT_TEMP_SENS_B_DIS         0x78  //120, Advanced Diagnostics A - IGBT temperature sensor B disable
    #define DIAG_AD_A_IGBT_TEMP_SENS_B_EN          0x79  //121, Advanced Diagnostics A - IGBT temperature sensor B enable
    #define DIAG_AD_A_IGBT_TEMP_SENS_C_DIS         0x7A  //122, Advanced Diagnostics A - IGBT temperature sensor C disable
    #define DIAG_AD_A_IGBT_TEMP_SENS_C_EN          0x7B  //123, Advanced Diagnostics A - IGBT temperature sensor C enable
    #define DIAG_AD_A_IGBT_TEMP_SENS_CHOP_DIS      0x88  //136, Advanced Diagnostics A - IGBT temperature sensor chopper disable
    #define DIAG_AD_A_IGBT_TEMP_SENS_CHOP_EN       0x89  //137, Advanced Diagnostics A - IGBT temperature sensor chopper enable

    #define DIAG_AD_A_BRAKE_RES_TEMP_SENS_DIS      0x8A  //138, Advanced Diagnostics A – Brake resistor temperature sensor disable
    #define DIAG_AD_A_BRAKE_RES_TEMP_SENS_EN       0x8B  //139, Advanced Diagnostics A - Brake resistor temperature sensor enable

    #define DIAG_AD_A_FUNCT_DIAG_SWITCH_TEST       0x8C  //140, Advanced Diagnostics A - Functional Diagnostics Switch Test

// Advanced Diagnostics B
    #define DIAG_AD_B_DO_NOTHING                   0xC8  //200, Advanced Diagnostics B - Do nothing

    #define DIAG_AD_B_CABLE_ORIENT                 0xC9  //201, Advanced Diagnostics B - Cable Orientation

    #define DIAG_AD_B_BLEED_DOWN_VOLT              0xCC  //204, Advanced Diagnostics B - Bleed down voltage check

    #define DIAG_AD_B_MOTOR_POS_SENS_CALIB         0xCF  //207, Advanced Diagnostics B - Motor position sensor calibration

    #define DIAG_AD_B_GENER_POS_SENS_CALIB         0xD0  //208, Advanced Diagnostics B - Generator position sensor calibration

    #define DIAG_AD_B_PWR_DIAG_GENER_POS           0xD1  //209, Advanced Diagnostics B -  Power Diagnostics Generator Position Sensor/Cable Orientation Test

#define NUM_DIAG_STATES                            34

// DIAG_OFF not included, will check seperately. no need to run through this array if there is no diagnostic message.
unsigned int[] Diagnostics_Array {
    DIAG_AD_A_DO_NOTHING,
    DIAG_AD_A_CURR_SENS_ZERO_OFFSET_CALIB,
    DIAG_AD_A_PHASE_A_CURR_SENS_DIS,
    DIAG_AD_A_PHASE_A_CURR_SENS_EN,
    DIAG_AD_A_PHASE_B_CURR_SENS_DIS,
    DIAG_AD_A_PHASE_B_CURR_SENS_EN,
    DIAG_AD_A_PHASE_C_CURR_SENS_DIS,
    DIAG_AD_A_PHASE_C_CURR_SENS_EN,
    DIAG_AD_A_BRAKE_RES_CURR_SENS_DIS,
    DIAG_AD_A_BRAKE_RES_CURR_SENS_EN,
    DIAG_AD_A_WIND_TEMP_SENS_A_DIS,
    DIAG_AD_A_WIND_TEMP_SENS_A_EN,
    DIAG_AD_A_WIND_TEMP_SENS_B_DIS,
    DIAG_AD_A_WIND_TEMP_SENS_B_EN,
    DIAG_AD_A_WIND_TEMP_SENS_C_DIS,
    DIAG_AD_A_WIND_TEMP_SENS_C_EN,
    DIAG_AD_A_IGBT_TEMP_SENS_A_DIS,
    DIAG_AD_A_IGBT_TEMP_SENS_A_EN,
    DIAG_AD_A_IGBT_TEMP_SENS_B_DIS,
    DIAG_AD_A_IGBT_TEMP_SENS_B_EN,
    DIAG_AD_A_IGBT_TEMP_SENS_C_DIS,
    DIAG_AD_A_IGBT_TEMP_SENS_C_EN,
    DIAG_AD_A_IGBT_TEMP_SENS_CHOP_DIS,
    DIAG_AD_A_IGBT_TEMP_SENS_CHOP_EN,   
    DIAG_AD_A_BRAKE_RES_TEMP_SENS_DIS,
    DIAG_AD_A_BRAKE_RES_TEMP_SENS_EN,
    DIAG_AD_A_FUNCT_DIAG_SWITCH_TEST,
    DIAG_AD_B_DO_NOTHING,
    DIAG_AD_B_CABLE_ORIENT,
    DIAG_AD_B_BLEED_DOWN_VOLT,
    DIAG_AD_B_MOTOR_POS_SENS_CALIB,
    DIAG_AD_B_GENER_POS_SENS_CALIB,
    DIAG_AD_B_PWR_DIAG_GENER_POS
};