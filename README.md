This project is a smart IoT bikelight.
It consists of the a physcial front and back light, a mobile application and webbased backend.

The Bikelight utilizes an ESP32C3 and LoRa integrated chip HT-CT62.
The HT-CT62 chip is mounted on the ESP32 Dev Backplane (HT-CT-ESP_V3), from Heltec. https://heltec.org/project/esp-dev-backplane/

This repository contains the arudino code for the bikelight and the schematic of the bikelight.

# Functionality of the bikelight

## Bikelight
The bikelight is battery driven and rechargeable.
It has three operation modes, Active, Park and Storage mode.
When connected to power the bikelight automatically goes to active mode, where it turns on when it is moved while it is dark. It then automatically turns off when the bike is parked.
It can be manually turned on and off with a press on a button. If the button is held down it is manually set to park mode.
By any time double pressing the button the five indicator LEDS show the battery level.

| Battery Level | Number of LEDs on |
|---------------|-------------------|
| 0-20%         | 1                 |
| 20-40%        | 2                 |
| 40-60%        | 3                 |
| 60-80%        | 4                 |
| 80-100%       | 5                 |

To put the bikelight in storage mode a switch must be flipped.

## Mobile Application
The mobile application can be downloaded from ....

In the mobile application, information from the bikelight can be seen.
This includea:
* The current battery level of the bikelight
* The current discharge rate / expected battery life time
* The last location of the bikelight
* The current data transfer period in the three operation modes
* Wheter the light is currently on or off.
* The current operation mode.

Furthermore, the data transfer periods can be updated from the app.
The ranges are:

| Operation Mode | Time intervals     | Default |
|----------------|--------------------|---------|
| Active         | 1 - 64 [Minuts]    | 2 min   |
| Park           | 1 - 64 [Hours]     | 8 h     |
| Switch to park | 10 - 640 [Seconds] | 30 s    |

The mobile app uses API calls to get data from the backend and enque data to the device.

## Backend
The LoRa communication is facilitated by Cibicom / Loriot.
From here the recieved data is distributed to AllThingsTalk, where it is decoded.

### Data transfer protocols
The device is always sending 53 bytes.

|Byte | 1 | 2 | 3 | 4 | 5-8 | 9-12 | 13-14 |15-16| 17-18 | 19-53 |
|-----| --| --| --| --| ----| ----| -------| ----| ------| ------|
|Content|Lights| operation mode| BatteryPercentage| Discharge rate|Latitude|Longitude| Active interval| Park Interval| Switch to park | Mac adresses|

The MACs are each 7 bytes long. The first byte being the signal strength (RSSI) and the rest being the actual MAC adress.




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
* 74HC595 Shift Register
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



