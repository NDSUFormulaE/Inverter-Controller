#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Get the first Arduino Mega port from arduino-cli board list
PORT=$(./bin/arduino-cli board list | grep "Arduino Mega" | awk '{print $1}')

if [ -z "$PORT" ]; then
    echo "No Arduino Mega found! Please check connection."
    exit 1
fi

echo "Found Arduino Mega at $PORT"

${SCRIPT_DIR}/compile_app.sh

if [ $? -eq 0 ]; then
    echo "Uploading to Arduino Mega..."
    ./bin/arduino-cli upload -p "$PORT" InverterApp
else
    echo "Compilation failed!"
    exit 1
fi
