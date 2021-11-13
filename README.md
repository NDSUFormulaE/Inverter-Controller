# Inverter-Controller
This repository is for the NDSU Formula Electric Inverter Controller and this system is designed to run on an Arduino Uno.

## Repo Structure
All important code is found inside the ``InverterApp``.
- ``InverterApp``: Top-Level Dir
    - ``InverterApp.ino``: main application file for programming the Arduino
    - ``src``: contains all source files
      - ``ARD1939``: Contains the ARD1939 library for base J1939 functionality
        - ``CAN_SPEC``: Contains header files from the John Deere PD400 Inverter's CAN Specification
      - ``TaskScheduler``: Contains our library for sending out scheduled CAN messages and interacting with CAN
- ``docs``: Contains all documentation from John Deere about the inverter and docs from the ARD1939 library    

## Guides
Check out this repos `wiki` for more details on setting up and building the application as well as general information on CAN
