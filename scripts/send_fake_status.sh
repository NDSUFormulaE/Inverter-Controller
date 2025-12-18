#!/bin/bash

# Sends fake inverter status messages at 15ms intervals
# Status messages are sent from the inverter (source address 0xA2)
# to the controller using J1939 CAN protocol

INTERVAL=0.015  # 15 ms between messages
CAN_INTERFACE="can0"

# Source address for inverter (0xA2)
SRC_ADDR="A2"

# J1939 CAN ID format: Priority (3 bits) | Reserved (1 bit) | DP (1 bit) | PGN (16 bits) | Source Addr (8 bits)
# Priority 6 (0b110) -> 0x18 in upper nibble for broadcast messages

# PGNs from PGN.h:
# STATUS1_RELTORQUE_SPEED   0xFFFE
# STATUS3_ABSTORQUE_SPEED   0xFFFB  
# STATUS4_TORQUE_PWRSTAGE_OVRLD 0xFFF4
# INVERTER_TEMP2_MACHINE    0xFFFF

# Build CAN IDs (29-bit extended): 0x18 + PGN + SRC
STATUS1_ID="18FFFE${SRC_ADDR}"      # PGN 0xFFFE
STATUS3_ID="18FFFB${SRC_ADDR}"      # PGN 0xFFFB
STATUS4_ID="18FFF4${SRC_ADDR}"      # PGN 0xFFF4
TEMP2_ID="18FFFF${SRC_ADDR}"        # PGN 0xFFFF

# Counter for varying fake data
counter=0

send_status_messages() {
    local cnt=$1
    
    # Vary some values based on counter for realistic-looking data
    local speed_low=$(printf "%02X" $((cnt % 256)))
    local speed_high=$(printf "%02X" $(((cnt / 2) % 256)))
    local torque_low=$(printf "%02X" $((128 + (cnt % 64))))
    local temp=$(printf "%02X" $((70 + (cnt % 20))))  # 30-50°C range (value + offset of -40)
    
    # STATUS1_RELTORQUE_SPEED (subtype 0x79): Torque and Speed
    # msg[0]=0x79, msg[1]=reserved, msg[2:3]=torque%, msg[4:5]=speed, msg[6:7]=reserved
    cansend ${CAN_INTERFACE} "${STATUS1_ID}#79FF${torque_low}80${speed_low}${speed_high}FFFF"
    
    # STATUS2_STATE_VOLTAGE (subtype 0x77, same PGN as STATUS1)
    # msg[0]=0x77, msg[1]=reserved, msg[2]=MCU_State, msg[3:4]=DC_Bus_Voltage, msg[5]=Derate, msg[6:7]=Diag
    cansend ${CAN_INTERFACE} "${STATUS1_ID}#77FF01C009000000"
    
    # PROGNOSTIC1_RMS_CURRENT (subtype 0x7A, same PGN as STATUS1)
    # msg[0]=0x7A, msg[1:2]=Phase A, msg[3:4]=Phase B, msg[5:6]=Phase C, msg[7]=Brake Current
    cansend ${CAN_INTERFACE} "${STATUS1_ID}#7A${torque_low}00${torque_low}00${torque_low}0005"
    
    # STATUS3_ABSTORQUE_SPEED (subtype 0x00 0x51): Absolute torque and speed
    # msg[0:1]=0x0051, msg[2:3]=torque, msg[4:5]=speed, msg[6:7]=reserved
    cansend ${CAN_INTERFACE} "${STATUS3_ID}#0051${torque_low}7D${speed_low}${speed_high}FFFF"
    
    # STATUS4_TORQUE_PWRSTAGE_OVRLD (subtype 0x32)
    # msg[0]=0x32, msg[1]=reserved, msg[2:3]=neg_torque, msg[4:5]=pos_torque, msg[6]=pwr_stage, msg[7]=overload
    cansend ${CAN_INTERFACE} "${STATUS4_ID}#32FF007D007D0100"
    
    # INVERTER_TEMP1_IGBT (subtype 0x90, same PGN as STATUS4)
    # msg[0]=0x90, msg[1:7]=IGBT temps (1-6) + brake chopper temp
    cansend ${CAN_INTERFACE} "${STATUS4_ID}#90${temp}${temp}${temp}${temp}${temp}${temp}${temp}"
    
    # INVERTER_TEMP2_MACHINE (subtype 0xE4): Motor and board temps
    # msg[0]=0xE4, msg[1:3]=motor temps, msg[4]=reserved, msg[5]=brake res, msg[6]=ctrl board, msg[7]=coolant
    cansend ${CAN_INTERFACE} "${TEMP2_ID}#E4${temp}${temp}${temp}FF${temp}${temp}${temp}"
}

echo "Sending fake inverter status messages on ${CAN_INTERFACE} at ${INTERVAL}s intervals"
echo "Press Ctrl+C to stop"

while true; do
    send_status_messages $counter
    
    counter=$((counter + 1))
    if [ $counter -ge 256 ]; then
        counter=0
    fi
    
    sleep $INTERVAL
done
