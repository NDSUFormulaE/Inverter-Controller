#!/bin/bash

REPO_ROOT=$(git -C "$(dirname "$0")" rev-parse --show-toplevel) || { echo "Not a git repository"; exit 1; }
cd "$REPO_ROOT"

# Compile and upload the InverterApp to Arduino Mega
echo "Compiling InverterApp..."
./bin/arduino-cli compile InverterApp
