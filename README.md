# WirelessSensor
Wireless Sensor Hobbyist Project using a Nordic nRF52840 board
and a Raspberry Pi computer.  The Nordic is attached to an
accelerometer and gyroscope, which is used for an AHRS (Attitude
and Heading Reference System).  The output from the AHRS is sent
over BLE from the Nordic to the Raspberry Pi where the object 
orientation is visualized using the Raspberry Pi graphics hardware.

TODO:
1) Refine AHRS calibration
- remove fixed constants
- require less operator intervention to do calibration
- Angles take a long time to stabilize, accelerometer readings?
2) Fix singularity at North and South Poles, can't get to 90 degree pitch
3) Fix Pitch sign reversal
4) Further testing
5) Nordic Timer to drive AHRS to improve accuracy?
6) Re-organize all code
- Place devi2c better, why does not work in constructor
7) Visualization code on Raspberry Pi
8) BLE code
- Advertising, or characteristic notification for sensor data?
- DFU Over the Air Updates
- Fix crashes associated with BLE 
- Fix lack of reliable start of APP with BLE
- Fix raspberry pi connection MTU issue
- cleanup BLE code to remove junk
- Remove / replace HRS and other remnants
9) BLE application
10) Battery Power
11) Prototype Board 
12) Remove requirements for Serial Port and SEGGER Debugger for general use
13) Calibration Storage, so you don't have to do calibration every time you power on
components/libraries/fds/fds.h
14) Add Doyxgen documentation for all code, procedures, and parameters
15) Add circuit schematic 
15) Add general documentation on the project, algorithms, calibration
16) Scrub Makefile
17) Turn BLE into C++ class code

