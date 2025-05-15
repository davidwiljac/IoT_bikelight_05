This project is a smart IoT bikelight.

The Bikelight utilizes an ESP32C3 and LoRa integrated chip HT-CT62.
The HT-CT62 chip is mounted on the ESP32 Dev Backplane (HT-CT-ESP_V3), from Heltec. https://heltec.org/project/esp-dev-backplane/

This repository contains the arudino code for the bikelight and the schematic of the bikelight.

# Functionality of the bikelight
The bikelight

# Software
To run the software, first follow Heltc's guide to setup the Heltec framework in the Arduino IDE.
The guide can be found here: https://docs.heltec.org/en/node/esp32/esp32_general_docs/quick_start.html

Secondly, all .ino-files and .h-files must be added to the same Arduino workspace.
This is done by:
1) Open main.ino, located in the folder "main", with Arduino IDE
2) Go to Sketch -> Add File...
3) Select all other .ino-files and .h-files files from the GitHub-folder "main"

The code can now be compiled and downloaded to the board.

# Hardware
The schematic of the bikelight, the HT-CT62 chip and the ESP32 Dev Backplane can be found in the repository.

To build the bikelight the following hardware is needed

* ESP32 Dev Backplane, with onbuilt HT-CT62 chip
* 2x Molex antennas
* Battery
* MAX17048 Battery Monitor
* ADXL345 Accelerometer
* Light Dependent Resistor (LDR)
* GY-NEO6MV2 GNSS module
* Push-botton
* Single pole double throw switch
* 5x LED
* Front- and backlight
* 2x NPN transistors
* 5x 10k $\Omega$ Resistors
* 5x 220 $\Omega$ Resistors
* 1x 100 $\Omega$ Resistor
* USB-C datacable
* Breadboard or solderboard
* Jumper Wires



