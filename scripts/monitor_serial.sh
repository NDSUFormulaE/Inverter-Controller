#!/bin/bash

# Get the first Arduino Mega port from arduino-cli board list
PORT=$(arduino-cli board list | grep "Arduino Mega" | awk '{print $1}')

if [ -z "$PORT" ]; then
    echo "No Arduino Mega found! Please check connection."
    exit 1
fi

echo "Found Arduino Mega at $PORT"

# Connect to Arduino's serial monitor
arduino-cli monitor -p "$PORT" -c baudrate=115200
