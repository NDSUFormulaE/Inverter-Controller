#!/bin/bash

REPO_ROOT=$(git -C "$(dirname "$0")" rev-parse --show-toplevel) || { echo "Not a git repository"; exit 1; }
cd "$REPO_ROOT"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source "$SCRIPT_DIR/arduino_cli.sh"

# Get the first Arduino Mega port from arduino-cli board list
PORT="${ARDUINO_PORT:-$(${ARDUINO_CLI_CMD[@]} board list | grep -Ei "arduino.*mega|mega 2560" | awk '{print $1}' | head -n 1)}"

if [ -z "$PORT" ]; then
    echo "No Arduino Mega found! Please check connection."
    exit 1
fi

echo "Found Arduino Mega at $PORT"

# Connect to Arduino's serial monitor
"${ARDUINO_CLI_CMD[@]}" monitor -p "$PORT" -c baudrate=115200
