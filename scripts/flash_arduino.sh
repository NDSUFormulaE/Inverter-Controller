#!/bin/bash

# Get the first Arduino Mega port from arduino-cli board list
PORT=$(arduino-cli board list | grep "Arduino Mega" | awk '{print $1}')

if [ -z "$PORT" ]; then
    echo "No Arduino Mega found! Please check connection."
    exit 1
fi

echo "Found Arduino Mega at $PORT"

# Compile and upload the InverterApp to Arduino Mega
echo "Compiling InverterApp..."
arduino-cli compile InverterApp

if [ $? -eq 0 ]; then
    echo "Uploading to Arduino Mega..."
    arduino-cli upload -p "$PORT" InverterApp
else
    echo "Compilation failed!"
    exit 1
fi
