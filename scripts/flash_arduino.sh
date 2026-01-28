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

${SCRIPT_DIR}/compile_app.sh

if [ $? -eq 0 ]; then
    echo "Uploading to Arduino Mega..."
    "${ARDUINO_CLI_CMD[@]}" upload -p "$PORT" InverterApp
else
    echo "Compilation failed!"
    exit 1
fi
