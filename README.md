# WirelessSensor
Wireless Sensor Hobbyist Project using a Nordic nRF52840

TODO:
1) Refine calibration
- remove fixed constants
- Angles take a long time to stabilize, accelerometer readings?
2) Fix singularity at North Pole, can't get to 90 degree pitch
3) Fix Pitch sign reversal
4) Further testing
5) Re-organize all code
- BLE in separate files from main.cpp
- Place devi2c better, why does not work in constructor
6) Visualization 
7) Re-enable BLE
8) BLE application
9) Battery Power
10) Prototype Board 
11) Remove requirements for Serial Port and SEGGER Debugger for general use
12) Calibration Storage, so you don't have to do calibration every time you power on

