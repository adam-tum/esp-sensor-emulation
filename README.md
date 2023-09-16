# Scalable Sensor Emulation for IoT Experiments

This repository contains code for a proof-of-concept implementation of an I2C based sensor emulation system on the ESP32.
It supports emulation of a single or multiple instances of a BMP180 sensor device on a single ESP32.

There are various configuration and testing functionalities provided within the code to test this system on multiple ESP32s simultaneously.

Some values, such as network information, MAC-addresses of connected ESPs, BMP180 configuration values, I2C timing parameters etc., may need to be adjusted. 

## Requirements
* ESP-IDF `v4.X`

## Building

 To clone this repository:
 
```
git clone https://github.com/adam-tum/esp-sensor-emulation.git
```

To build the ESP32 code [more information about build system and project structure](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/):

```
idf.py build
```
