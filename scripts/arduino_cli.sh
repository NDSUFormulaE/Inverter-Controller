#!/bin/bash

REPO_ROOT=$(git -C "$(dirname "$0")" rev-parse --show-toplevel) || { echo "Not a git repository"; exit 1; }

ARDUINO_CLI=""
if [ "$(uname -s)" = "Darwin" ] && [ -x "$REPO_ROOT/bin/arduino-cli-darwin" ]; then
    ARDUINO_CLI="$REPO_ROOT/bin/arduino-cli-darwin"
else
    ARDUINO_CLI=$(command -v arduino-cli)
    if [ -z "$ARDUINO_CLI" ] || [ ! -x "$ARDUINO_CLI" ]; then
        ARDUINO_CLI="$REPO_ROOT/bin/arduino-cli"
    fi
fi

if [ -z "$ARDUINO_CLI" ] || [ ! -x "$ARDUINO_CLI" ]; then
    echo "arduino-cli not found. Install it or place it at ./bin/arduino-cli (Linux) or ./bin/arduino-cli-darwin (macOS)"
    exit 1
fi

ARDUINO_CLI_CMD=("$ARDUINO_CLI")
