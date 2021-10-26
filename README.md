# Inverter-Controller
This repository is for the NDSU Formula Electric Inverter Controller and this system is designed to run on an Arduino Uno.

## Setup
- Download the Arduino IDE from [here](https://www.arduino.cc/download_handler.php?f=/arduino-1.8.13-windows.exe).
- Clone this repository
- Open ``InverterApp/InverterApp.ino`` in the ``Arduino IDE``.
 - This application requires the ``TimeLib`` library to install click ``Tools`` in top bar, then click ``Manage Libraries``.
 - Search for ``TimeLib`` and install the library from Michael Margolis.
- You should be able to run the ``Verify`` and ``Upload`` buttons at the top of the IDE without any errors.

## Repo Structure
All important code is found inside the ``InverterApp``.
- ``InverterApp``: Top-Level Dir
    - ``InverterApp.ino``: main application file for programming the Arduino
    - ``src``: contains all source files
      - ``ARD1939``: Contains the ARD1939 library for base J1939 functionality
        - ``CAN_SPEC``: Contains header files from the John Deere PD400 Inverter's CAN Specification
      - ``TaskScheduler``: Contains our library for sending out scheduled CAN messages and interacting with CAN
- ``docs``: Contains all documentation from John Deere about the inverter and docs from the ARD1939 library    
