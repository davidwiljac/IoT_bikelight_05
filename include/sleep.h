#include <stdint.h>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "main.h"
void modem_sleep(int32_t time_millis);
void modem_wake();
void esp_sleep(uint8_t mode, uint64_t *GPSInterval);