// Array of all variables pulled from CAN Messages
// Brennan

#include <inttypes.h>

struct CANVariables
{
    float Avg_Torque_Percent;
    float Rel_Machine_Speed;
    uint32_t MCU_State;
    float DC_Bus_Voltage;
    uint32_t Derate_Owner;
    uint32_t Diag_Function;
    uint32_t Diag_Status;
    float RMS_Current_Phase_A;
    float RMS_Current_Phase_B;
    float RMS_Current_Phase_C; 
    uint32_t Brake_Resistor_RMS_Current;
    float Brake_Resistance;
    float DC_Link_Capacitance;
    float Motor_BEMF;
    uint32_t EMI_Capacitance;
    float Machine_Speed_200ms_Avg;
    float Mach_Torq_Percent_200ms_Avg;
    float Stored_Pos_Offset;
    float Calculated_Pos_Offset;
    float Avg_Abs_Torque;
    float Abs_Machine_Speed;
    float Actual_Power;
    float Max_Power_Generating;
    float Max_Power_Motoring;
    float RMS_Voltage_Phase_A;
    float RMS_Voltage_Phase_B;
    float RMS_Voltage_Phase_C;
    float Neg_Torque_Available;
    float Pos_Torque_Available;
    uint32_t Power_Stage_Status;
    float Overload_Percent;
    int32_t IGBT1_Temp;
    int32_t IGBT2_Temp;
    int32_t IGBT3_Temp;
    int32_t IGBT4_Temp;
    int32_t IGBT5_Temp;
    int32_t IGBT6_Temp;
    int32_t Brake_Chopper_IGBT_Temp;
    uint32_t AC_Voltage_Output;
    uint32_t AC_Frequency;
    uint32_t AC_Voltage_Desired;
    float Actual_Current;
    float Max_Current_Generating;
    float Max_Current_Motoring;
    int32_t Motor_Temp_1;
    int32_t Motor_Temp_2;
    int32_t Motor_Temp_3;
    int32_t Brake_Resistor_Temp;
    int32_t Control_Board_Temp;
    int32_t Inverter_Coolant_Temp;
    uint32_t Protect_Lamp_Status;
    uint32_t Amber_Warning_Lamp_Status;
    uint32_t Red_Stop_Lamp_Status;
    uint32_t Multi_Indicator_Lamp_Status;
    uint32_t Flash_Protect_Lamp_Status;
    uint32_t Flash_Amber_Warning_Lamp_Status;
    uint32_t Flash_Red_Stop_Lamp_Status;
    uint32_t Flash_Multi_Indicator_Lamp_Status;
    uint32_t DM1_SPN;
    uint32_t DM1_FMI;
    uint32_t Occurrence_Count;
};

struct CANVariables InverterState = {};