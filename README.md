# WirelessSensor
Wireless Sensor Hobbyist Project using a Nordic nRF52840

TODO:
1) Refine calibration
- remove fixed constants
- require less operator intervention to do calibration
- Angles take a long time to stabilize, accelerometer readings?
2) Fix singularity at North and South Poles, can't get to 90 degree pitch
3) Fix Pitch sign reversal
4) Further testing
5) Nordic Timer to drive AHRS to improve accuracy?
6) Re-organize all code
- BLE in separate files from main.cpp
- Place devi2c better, why does not work in constructor
7) Visualization 
8) BLE code
- Re-enable BLE 
- DFU Over the Air Updates
- Fix crashes associated with BLE 
- Fix lack of reliable start of APP with BLE
- Fix raspberry pi connection MTU issue
- cleanup BLE code to remove junk
9) BLE application
10) Battery Power
11) Prototype Board 
12) Remove requirements for Serial Port and SEGGER Debugger for general use
13) Calibration Storage, so you don't have to do calibration every time you power on
components/libraries/fds/fds.h
