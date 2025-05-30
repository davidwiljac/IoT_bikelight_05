This project is part of the DTU Course: _34346, Networking technologies and application development for Internet of Things (IoT), spring 2025_.

# Flashback - A Smart IoT Bike Light

<img src="https://github.com/user-attachments/assets/b148089f-7273-476c-a812-d38bbbe342ea" alt="Flashback Bike Light" width="400"/>
<img src="https://github.com/user-attachments/assets/243edefd-8c1a-42ab-8936-3a1d32cae373" alt="front light" width="317"/>


FlashBack consists of the a physcial front and back light, a mobile application and webbased backend.

The bike light utilizes the HT-CT62 chip, which contain an ESP32C3 and an SX1262 LoRa-module.

The HT-CT62 chip is mounted on the ESP32 Dev Backplane (HT-CT-ESP_V3), from Heltec. https://heltec.org/project/esp-dev-backplane/

This repository contains the arudino code for the bike light and the schematic of the bike light.

Arduino code in folder: "main"

KiCAD Schematic in folder: "FlashBack_Schematic"

# Functionality of the bike light

## Bike light
The bike light is battery driven and rechargeable.

It has three operation modes, Active, Park and Storage mode.

When connected to power the bike light automatically goes to active mode, where it turns on when it is moved while it is dark. It then automatically turns off when the bike is parked.

It can be manually turned on and off with a press on a button. If the button is held down it is manually set to park mode.
By any time double pressing the button the five indicator LEDS show the battery level.

| Battery Level | Number of LEDs on |
|---------------|-------------------|
| 0-20%         | 1                 |
| 20-40%        | 2                 |
| 40-60%        | 3                 |
| 60-80%        | 4                 |
| 80-100%       | 5                 |

To put the bike light in storage mode a switch must be flipped.

## Mobile Application
The mobile application can be downloaded from: https://drive.google.com/file/d/12I2f2bF7VSXXHKXbHlzPAXHOnTvUsZBw/view?usp=sharing

The GitHub Repository for the app can be found here: https://github.com/4444T7777/Flashback_IoT

In the mobile application, information from the bike light can be seen.

This include:
* The current battery level of the bike light
* The last location of the bike light
* The current data transfer period in the three operation modes
* Wheter the light is currently on or off.
* The current operation mode.

Furthermore, the data transfer periods can be updated from the app.

The ranges are:

| Operation Mode | Time intervals     | Default |
|----------------|--------------------|---------|
| Active         | 1 - 63 [Minutes]   | 2 min   |
| Park           | 1 - 63 [Hours]     | 8 h     |
| Switch to park | 10 - 630 [Seconds] | 30 s    |

The mobile app uses API calls to get data from the backend and enque data to the device.

## Backend
The LoRa communication is facilitated by Cibicom / Loriot.
From here the recieved data is distributed to AllThingsTalk, where it is decoded.

### Data transfer protocols

#### Uplink
The device is always sending 53 bytes.

|Byte: | 1 | 2 | 3 | 4 | 5-8 | 9-12 | 13-14 |15-16| 17-18 | 19-158 |
|-----| --| --| --| --| ----| ----| -------| ----| ------| ------|
|Content:|Lights On/Off| Operation mode| BatteryPercentage| Discharge rate|Latitude|Longitude| Active interval| Park Interval| Switch to park | MACs|

**Operation mode**

A number indicating the mode:

1: Active, 2: Park , 3: Storage

**MACs**

The bikelight makes a WiFi-scan and sends the MAC adresses and signal strenghts of the five most strongest access points.
The MACs are therefore each 7 bytes long. The first byte being the signal strength (RSSI) and the rest being the actual MAC adress.

#### Downlink
There can be sent up to 3 bytes data to the device at a time.
Each byte is organized as follows:

|Bit: | 1-2 | 3-8 |
|-----| --- | --- |
|Content:| Mode to be set | Number |

**Mode to be set**

01: Active

10: Park

11: Switch to park time

**Number**

An integer from 1-63. This number is then intepreted accordingly to mode


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
To build the bike light the following hardware is needed:

* ESP32 Dev Backplane, with onbuilt HT-CT62 chip
* 2x Molex antennas
* Rechargeable Lithium-ion Battery and battery holder
* MAX17048 Battery Monitor
* ADXL345 Accelerometer
* Light Dependent Resistor (LDR)
* GY-NEO6MV2 GNSS module
* SN74HC595 Shift Register
* Push-botton
* Single pole double throw switch
* 5x LED
* Front- and backlight
* 2x NPN transistors
* 5x 10kOhm resistors
* 5x 220 Ohm resistors
* 1x 100 Ohm resistor
* USB-C datacable
* Breadboard or solderboard
* Jumper Wires

The schematic of our design can be seen below.

![billede](https://github.com/user-attachments/assets/1f1d4ea3-5217-459d-bb75-25cb235001ce)



The Battery and power management of the ESP32 Dev Backplane has been included in the schematic.
The USB-C interface and management have however not been included.

Moreover, it has been indicated on the HT-CT62 chip, which GPIOs are used for the internal LoRa connections.







