//PD400 CAN Spec 2.3.3 State Transition Command
//Brennan

#define NO_CHANGE                                  0x00  //00, No Change
#define STDBY_TO_FUNCTIONAL_DIAG                   0x03  //03, Standby to Functional Diagnostics
#define PWR_READY_TO_PWR_DIAG                      0x06  //06, Power Ready to Power Diagnostics
#define DRIVE_READY_TO_NORM_OPS                    0x08  //08, Drive Ready to Normal Operation
#define NORM_OPS_TO_DISCHARGE_DIAG                 0x09  //09, Normal Operation to Discharge Diagnostics
#define FAULT_CLASSA_TO_STDBY                      0x0F  //15, Fault Class A to Standby
#define IGNIT_READY_TO_ADV_DIAG_CLASSA             0x10  //16, Ignition Ready to Advanced Diagnostics Class A
#define FAULT_CLASSA_TO_ADV_DIAG_CLASSA            0x11  //17, Fault Class A to Advanced Diagnostics Class A
#define FAULT_CLASSB_TO_PWR_READY                  0x16  //22, Fault Class B to Power Ready
#define NORM_OPS_TO_DRIVE_READY                    0x17  //23, Normal Operation to Drive Ready
#define PWR_READY_TO_ADV_DIAG_CLASSB               0x18  //24, Power Ready to Advanced Diagnostics Class B
#define FAULT_CLASSB_TO_ADV_DIAG_CLASSB            0x19  //25, Fault Class B to Advanced Diagnostics Class B
#define DRIVE_READY_TO_ADV_DIAG_CLASSB             0x1A  //26, Drive Ready to Advanced Diagnostics Class B
#define FAULT_CLASSB_TO_FAIL_SAFE                  0x1D  //29, Class B to Fail Safe
#define FAULT_CLASS_B_ADV_DIAG_CLASSA_TO_FAIL_SAFE 0x1F  //31, Fault Class B/Advanced Diagnostics Class B to Fail Safe
#define PWR_READY_TO_DRIVE_READY                   0x23  //35, Power Ready to Drive Ready
#define STDBY_TO_ADV_DIAG_CLASSA                   0x25  //37, Standby to Advanced Diagnostics Class A
#define STDBY_TO_IGNIT_READY                       0x26  //38, Standby to Ignition Ready
#define ADV_DIAG_CLASSB_TO_PWR_READY               0x5B  //91, Advanced Diagnostics Class B to Power Ready
#define ADV_DIAG_CLASSA_TO_STDBY                   0x5C  //92, Advanced Diagnostics Class A to Standby
#define FAULT_CLASSB_TO_STDBY                      0x5D  //93, Fault Class B to Standby
#define IGNIT_READY_TO_STDBY                       0x5E  //94, Ignition Ready to Standby
#define PWR_READY_TO_STDBY                         0x5F  //95, Power Ready to Standby
#define DRIVE_READY_TO_STDBY                       0x60  //96, Drive Ready to Standby
#define NORM_OPS_TO_STDBY                          0x61  //97, Normal Operation to Standby
#define NORM_OPS_TO_PWR_READY                      0x62  //98, Normal Operation to Power Ready
#define DRIVE_READY_TO_PWR_READY                   0x63  //99, Drive Ready to Power Ready

#define NUM_STATE_TRANSITIONS                      27