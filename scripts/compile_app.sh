#!/bin/bash

REPO_ROOT=$(git -C "$(dirname "$0")" rev-parse --show-toplevel) || { echo "Not a git repository"; exit 1; }
cd "$REPO_ROOT"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Compile and upload the InverterApp to Arduino Mega
echo "Compiling InverterApp..."

source "$SCRIPT_DIR/arduino_cli.sh"

"${ARDUINO_CLI_CMD[@]}" compile InverterApp
