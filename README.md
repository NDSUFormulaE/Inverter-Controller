############
PD400Library
############

Library for controlling John Deere Electronic Systems PD400 - Single Inverter using an Arduino w/ CANShield.

*****
Setup
*****
- Add ``PD400`` and ``ARD1939`` Libraries to libraries in Arduino IDE.

*********
Reference
*********

State Transitions
=================

CAN Timeout
===========


Standard J1939 PGN's
====================

- ``0xFEDA`` *(65242)* **Software ID PGN**::
 
    Contains Software Part Number (PN) and revision number information.
    
- ``0xFDC5`` *(64965)* **ECU ID PGN**::

    Contains the Hardware Part Number, Serial Number, and config information.
    
- ``0xFEEB`` *(65259)* **Component ID PGN**::
 
    Contains info on current application configuration.
    
CAN Command Messages
====================
The inverter can be commanded in 4 modes. Relative Torque Mode, Speed Mode, Voltage Mode, and Absolute Torque Mode. They each have their own corresponding command message.

**CAN Timeout**::

   20 milliseconds without a valid command message will cause the inverter to enter a fault state
   
.. image:: docs/relTorque.png

.. image:: docs/speedMode.png

.. image:: docs/voltageMode.png

.. image:: docs/absTorque.png
