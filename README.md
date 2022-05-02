# WirelessSensor
Wireless Sensor Hobbyist Project using a Nordic nRF52840 Dongle
and a Raspberry Pi computer.  The Nordic chip is attached to an
accelerometer, magnetometer and gyroscope, which is used for an 
AHRS (Attitude and Heading Reference System).  The output from 
the AHRS is sent over Bluetooth Low Energy (BLE) from the Nordic 
to the Raspberry Pi where the object orientation is visualized 
using the Raspberry Pi graphics hardware, and control of the 
AHRS parameter settings can be interactively changed using a 
Python-based GUI.

# Hardware Requirements

1. [Nordic Semiconductor nRF52840 Dongle](https://www.mouser.com/ProductDetail/949-NRF52840-DONGLE)
2. [Adafruit 4485 -- LSM6DS33 + LIS3MDL - 9 DoF IMU with Accel / Gyro / Mag](https://www.adafruit.com/product/4485) or equivalent
3. Raspberry Pi3B (may work with other Pis)
4. Personal computer, Windows or Mac, with USB-A Connector
5. Optional: [Adafruit #3309 - CP2104 Friend](https://www.adafruit.com/product/3309) - USB to Serial Converter or equivalent, for Serial Console to Nordic chip.

# Software Requirements

1. [Nordic Semiconductor nRF52 SDK v16.0.0 or later](https://www.nordicsemi.com/Products/Development-software/nrf5-sdk/download)
2. [Nordic recommended Development IDE](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fug_nrf52840_dk%2FUG%2Fcommon%2Fnordic_tools.html).  One of the following:
   - [GNU/GCC](https://gcc.gnu.org/) (Recommended for this project, version 9.2 or later).
   - [SEGGER Embedded Studio (SES)](https://www.segger.com/products/development-tools/embedded-studio/)
   - [MDK-ARM Keil µVision](https://www2.keil.com/mdk5/uvision/)
   - [IAR](https://www.iar.com/iar-embedded-workbench/#!?architecture=ARM)
3. Python3 for Raspberry Pi 
4. [BlueZ](http://www.bluez.org/) v5.47 or later for Raspberry Pi
5. [Bluepy](https://github.com/IanHarvey/bluepy/tree/v/1.3.0) v1.3 or later, for Raspberry Pi

The Software development platform I've used for the Nordic Semiconductor Board is an x86 Windows PC which has Cygwin installed.  [Cygwin](https://www.cygwin.com/) is a software layer which runs on top of Windows 10/11 and provides functionality similar to a Linux distribution.  You can compile the Nordic Semiconductor source code once you have installed the GNU GCC toolchain (using Cygwin's package manager). It's a very simple way to run Windows and Linux applications on the same box.

## Software Installation Steps

## Nordic SDK Installation

In the Nordic nRF52 SDK, edit the file 
components/toolchain/gcc/Makefile.<platform> 
and change the following Makefile variables to point to your installation 
and version of the ARM gcc compiler (Windows example shown).

GNU_INSTALL_ROOT := C:/Program Files (x86)/GNU Tools ARM Embedded/9 2019-q4-major/bin/

GNU_VERSION := 9.2.1

## Nordic Software Compilation and Download

1. cd into the Directory Sensor/src
2. Type 'make'
3. Install Nordic firmware onto board using nRF Connect for Desktop via USB or SEGGER debugger

# Remaining Improvements:
1. Refine AHRS calibration
   - Improve calibration algorithm for accelerometer
   - remove fixed constants
   - require less operator intervention to do calibration
   - Angles take a while to stabilize
2. Fix singularity at North and South Poles, can't get to 90 degree pitch (gimbal lock)
3. Fix Pitch sign reversal
4. Further testing
5. Nordic Timer to drive AHRS to improve accuracy?  sampleFreq needs to be set right.
6. Re-organize all code
   - Place devi2c better, why does not work in constructor
7. BLE code
   - DFU Over the Air Updates
   - cleanup BLE code to remove junk
   - Remove / replace HRS and other remnants
   - Improve performance, being aware of battery life
8. Battery Power
9. Prototype Board 
10. Calibration Storage, so you don't have to do calibration every time you power on
components/libraries/fds/fds.h
11. Add Doyxgen documentation for all code, procedures, and parameters
12. Add circuit schematic 
13. Add general documentation on the project, algorithms, calibration
14. Develop Mechanical Test fixture to hold board and measure actual orientation to compare against calculated orientation.
15. Improve Calibration and Control GUI

