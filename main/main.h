#include <Arduino.h>
#include <LinkedList.h>
#include "Adafruit_MAX1704X.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL343.h"
// #include <TinyGPSPlus.h>

#include "HT_TinyGPS++.h"

#include <SoftwareSerial.h>
#include "LoRaWan_APP.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

bool readButton();
bool toggleButtonState(bool buttonState, LinkedList<bool> buttonStateList, bool button_has_been_released);
void active();
void park();
void storage();
void int1_isr(void);