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
ADDR_CLAIM_ID="18EE00${SRC_ADDR}"   # PGN 0xEE00 (Address Claim)

# J1939 NAME for fake inverter (8 bytes)
# This is a fake NAME identifying the simulated inverter
INVERTER_NAME="00A0000000000000"

# Counter for varying fake data
counter=0

# MCU States array (normal operational flow)
# STDBY=0x00, FUNC_DIAG=0x01, IGNIT_READY=0x03, PWR_READY=0x04, DRIVE_READY=0x06, NORM_OPS=0x08
MCU_STATES=(00 01 03 04 06 08)
state_index=0

# Fault state tracking
fault_active=0
fault_cycle=0
FAULT_ON_CYCLES=200   # ~3 seconds with faults active
FAULT_OFF_CYCLES=400  # ~6 seconds without faults

send_address_claim() {
    # Send address claim response from inverter
    cansend ${CAN_INTERFACE} "${ADDR_CLAIM_ID}#${INVERTER_NAME}"
}

# Send DM1 fault message using J1939 Transport Protocol
# DM1 PGN = 0xFECA, but uses TP for multi-packet
send_dm1_fault() {
    local occurrence=$1
    local occ_hex=$(printf "%02X" $occurrence)
    
    # Transport Protocol - Connection Management (TP.CM_BAM)
    # PGN 0xECFF = broadcast announcement
    # Data: 0x20 (BAM), total bytes (10), num packets (2), 0xFF, PGN LSB, PGN mid, PGN MSB
    cansend ${CAN_INTERFACE} "1CECFF${SRC_ADDR}#200A0002FFCAFE00"
    sleep 0.001
    
    # Transport Protocol - Data Transfer (TP.DT) packet 1
    # Sequence 1, then DM1 data: lamp status (0xFF=all off), SPN (3 bytes), FMI, occurrence
    # Example fault: SPN 520231 (0x7F027), FMI 31
    cansend ${CAN_INTERFACE} "1CEBFF${SRC_ADDR}#01FFFFA7FAE81FA2"
    sleep 0.001
    
    # TP.DT packet 2 - continuation
    # SPN 521699 (0x7F5E3), FMI 25, occurrence
    cansend ${CAN_INTERFACE} "1CEBFF${SRC_ADDR}#02E3E919${occ_hex}FFFFFFFF"
}

# Send DM1 with no active faults (lamp status all off, no DTCs)
send_dm1_clear() {
    # Single frame DM1 - no faults
    # PGN 0xFECA, 8 bytes: lamp status (6 bytes 0xFF = no lamps), then 0xFF padding
    cansend ${CAN_INTERFACE} "18FECA${SRC_ADDR}#FFFFFFFFFFFFFFFF"
}

send_status_messages() {
    local cnt=$1
    local mcu_state=$2
    
    # Vary some values based on counter for realistic-looking data
    # RPM: 0-3000 RPM, conversion: RPM = raw * 0.5 - 16000
    # 0 RPM -> raw = 32000 (0x7D00), 3000 RPM -> raw = 38000 (0x9470)
    local speed_raw=$(( 32000 + (cnt * 23) % 6001 ))  # Cycles through 32000-38000
    local speed_low=$(printf "%02X" $((speed_raw % 256)))
    local speed_high=$(printf "%02X" $((speed_raw / 256)))
    local torque_low=$(printf "%02X" $((128 + (cnt % 64))))
    local temp=$(printf "%02X" $((70 + (cnt % 20))))  # 30-50°C range (value + offset of -40)
    
    # DC Bus Voltage: 0-400V, conversion factor 0.03125 -> raw 0-12800 (0x0000-0x3200)
    local voltage_raw=$(( (cnt * 50) % 12801 ))  # Cycles through 0-12800
    local voltage_low=$(printf "%02X" $((voltage_raw % 256)))
    local voltage_high=$(printf "%02X" $((voltage_raw / 256)))
    
    # STATUS1_RELTORQUE_SPEED (subtype 0x79): Torque and Speed
    # msg[0]=0x79, msg[1]=reserved, msg[2:3]=torque%, msg[4:5]=speed, msg[6:7]=reserved
    cansend ${CAN_INTERFACE} "${STATUS1_ID}#79FF${torque_low}80${speed_low}${speed_high}FFFF"
    
    # STATUS2_STATE_VOLTAGE (subtype 0x77, same PGN as STATUS1)
    # msg[0]=0x77, msg[1]=reserved, msg[2]=MCU_State, msg[3:4]=DC_Bus_Voltage, msg[5]=Derate, msg[6:7]=Diag
    cansend ${CAN_INTERFACE} "${STATUS1_ID}#77FF${mcu_state}${voltage_low}${voltage_high}000000"
    
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

# Send initial address claim
echo "Sending address claim from inverter (SA: 0x${SRC_ADDR})..."
send_address_claim

while true; do
    send_status_messages $counter ${MCU_STATES[$state_index]}
    
    # Re-send address claim every 256 cycles (~4 seconds)
    if [ $((counter % 256)) -eq 0 ]; then
        send_address_claim
    fi
    
    # Cycle MCU state every 67 cycles (~1 second)
    if [ $((counter % 67)) -eq 0 ]; then
        state_index=$(( (state_index + 1) % ${#MCU_STATES[@]} ))
    fi
    
    # Fault cycling logic - send faults for a while, then clear them
    fault_cycle=$((fault_cycle + 1))
    if [ $fault_active -eq 1 ]; then
        # Send fault every 10 cycles while active (~150ms)
        if [ $((counter % 10)) -eq 0 ]; then
            send_dm1_fault $((counter % 255 + 1))
        fi
        # Turn off faults after FAULT_ON_CYCLES
        if [ $fault_cycle -ge $FAULT_ON_CYCLES ]; then
            echo "Clearing faults..."
            send_dm1_clear
            fault_active=0
            fault_cycle=0
        fi
    else
        # Turn on faults after FAULT_OFF_CYCLES
        if [ $fault_cycle -ge $FAULT_OFF_CYCLES ]; then
            echo "Activating faults..."
            fault_active=1
            fault_cycle=0
        fi
    fi
    
    counter=$((counter + 1))
    if [ $counter -ge 65536 ]; then
        counter=0
    fi
    
    sleep $INTERVAL
done
