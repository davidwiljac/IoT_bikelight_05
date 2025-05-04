#include <Arduino.h>
#include <LinkedList.h>
#include "Adafruit_MAX1704X.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL343.h"
#include "driver/gpio.h"
#include "HT_TinyGPS++.h"

#include <SoftwareSerial.h>
#include "LoRaWan_APP.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "tracking.h"

// Constants
#define GPS_interval_active 30 * 1000   // 30 seconds
#define GPS_interval_parked 120 * 1000 // 2 minutes
#define GPS_interval_storage 30 * 1000 // 30 seconds
#define GPS_interval_timeout 60 * 1000 // 1 minute
#define GPS_interval_retry  5 * 1000 // 5 seconds

#define GPS_max_tries 5            // Max tries to get a fix
#define WIFI_max_tries 5           // Max tries to get a fix

#define swtich_to_park_time 1000 * 1000 // 60 seconds
#define swtich_to_storage_time 20000 * 1000 // 20 seconds

#define sleep_time_active 1 * 100000   // 1 second
#define sleep_time_park 10 * 1000000   // 10 seconds
#define sleep_time_storage 30 * 1000000 // 30 seconds

bool readButton();
bool toggleButtonState(bool buttonState, LinkedList<bool> buttonStateList, bool button_has_been_released);
void active();
void park();
void storage();
void int1_isr(void);