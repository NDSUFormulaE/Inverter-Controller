// CAN Command Messages (PD400 CAN Spec 2.3.1.X) -> Sent to Inverter
#define COMMAND1_RELTORQUE            0xEF00      // must be |'d with Inverter Source Address
#define COMMAND2_SPEED                0xEF00      // must be |'d with Inverter Source Address
#define COMMAND3_VOLTAGE              0xEF00      // must be |'d with Inverter Source Address
#define COMMAND7_ABSTORQUE            0xEF00      // must be |'d with Inverter Source Address

#define BRAKE_RESISTOR                0xFFFE
#define TORQUE_LIMITING               0xFFFE
#define BUS_DISSIPATION               0xEF00      // must be |'d with Inverter Source Address
#define AC_SUPPLY_COMMAND             0xEF00      // must be |'d with Inverter Source Address
#define AC_SUPPLY_LIMITS              0xEF00      // must be |'d with Inverter Source Address
#define THREE_PHASE_SHORT             0xEF00      // must be |'d with Inverter Source Address
#define DC_LINK_PWR_LIMITING          0xEF00      // must be |'d with Inverter Source Address
#define DC_LINK_PWR_CURRENT_LIMITING  0xEF00      // must be |'d with Inverter Source Address

// CAN Status Messages (PD400 CAN Spec 2.3.2.X) -> Sent From Inverter
#define STATUS1_RELTORQUE_SPEED       0xFFFE
#define STATUS2_STATE_VOLTAGE         0xFFFE
#define STATUS3_ABSTORQUE_SPEED       0xFFFB
#define STATUS4_TORQUE_PWRSTAGE_OVRLD 0xFFF4

#define INVERTER_TEMP1_IGBT           0xFFFE
#define INVERTER_TEMP2_MACHINE        0xFFFF

#define PROGNOSTIC1_RMS_CURRENT       0xFFFE 
#define PROGNOSTIC2_DIAGNOSTIC        0xFFFE
#define PROGNOSTIC3_DIAGNOSTIC        0xFFFE
#define PROGNOSTIC5_POSITION          0xFFFE

#define AC_SUPPLY_STATUS              0xFFF4
#define DC_LINK_PWR_STATUS            0xFFFB
#define DC_LINK_PWR_CURRENT_STATUS    0xFFF4
#define VOLTAGE_RMS1                  0xFFFB

#define DM1                           0xFECA
