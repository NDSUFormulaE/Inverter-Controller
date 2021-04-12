//PD400 CAN Spec 2.3.4 Motor Control Unit State Definitions
//Brennan

#define MCU_PWR_UP                   0x00  //00, Power Up
#define MCU_STDBY                    0x00  //00, Standby
#define MCU_FUNCTIONAL_DIAG          0x01  //01, Functional Diagnostics
#define MCU_FAULT_CLASSA             0x02  //02, Fault Class A
#define MCU_IGNIT_READY              0x03  //03, Ignition Ready
#define MCU_PWR_READY                0x04  //04, Power Ready
#define MCU_PWR_DIAG                 0x05  //05, Power Diagnostics
#define MCU_DRIVE_READY              0x06  //06, Drive Ready
#define MCU_NORM_OPS                 0x08  //08, Normal Operation
#define MCU_FAULT_CLASSB             0x09  //09, Fault Class B
#define MCU_CNTRL_PWR_DOWN           0x0A  //10, Controlled Power Down
#define MCU_FAIL_SAFE                0x0B  //11, Fail Safe
#define MCU_ADV_DIAG_CLASSA          0x0D  //13, Advanced Diagnostics Class A
#define MCU_DISCHARGE_DIAG           0x0F  //15, Discharge Diagnostics
#define MCU_ADV_DIAG_CLASSB          0x11  //17, Advanced Diagnostics Class B

#define NUM_MCU_STATES               15

unsigned int[] MCU_State_Array = {
    MCU_PWR_UP,
    MCU_STDBY,
    MCU_FUNCTIONAL_DIAG,
    MCU_FAULT_CLASSA,
    MCU_IGNIT_READY,
    MCU_PWR_READY,
    MCU_DRIVE_READY,
    MCU_NORM_OPS,
    MCU_FAULT_CLASSB,
    MCU_CNTRL_PWR_DOWN,
    MCU_FAIL_SAFE,
    MCU_ADV_DIAG_CLASSA,
    MCU_DISCHARGE_DIAG,
    MCU_ADV_DIAG_CLASSB
};
