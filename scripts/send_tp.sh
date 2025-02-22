#!/bin/bash

# Configuration
INTERVAL=0.005  # Time between fault messages in seconds
MAX_OCCURRENCES=255  # Maximum number of occurrences before resetting
OCCURRENCE_INCREMENT=1  # How much to increment occurrence each time

# Initialize occurrence counter
occurrence=1

# Function to send DM1 message using transport protocol
send_dm1() {
    local occ=$1
    # Convert occurrence to hex
    local occ_hex=$(printf "%02X" $occ)
    
    # Send transport protocol messages in sequence
    cansend can0 "1CECFFA2#200A0002FFCAFE00" 
    sleep 0.001
    cansend can0 "1CEBFFA2#01FFFFA7FAE81FA2"
    sleep 0.001
    cansend can0 "1CEBFFA2#02E3E9${occ_hex}FFFFFFFFF"
    sleep 0.001
}

echo "Starting DM1 transport protocol message loop. Press Ctrl+C to stop."

while true; do
    send_dm1 $occurrence
    
    # Increment occurrence, reset if we hit max
    occurrence=$((occurrence + OCCURRENCE_INCREMENT))
    if [ $occurrence -gt $MAX_OCCURRENCES ]; then
        occurrence=1
    fi
    
    sleep $INTERVAL
done
