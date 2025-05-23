#include "sleep.h"

//modem sleep: Turns off the wifi and Bluetooth modules
void modem_sleep(int32_t time_millis) {
  WiFi.setSleep(true);
  btStop();
  esp_wifi_set_mode(WIFI_MODE_NULL);
}

//modem wake: Turns on WiFi and Bluetooth modules
void modem_wake() {
  esp_wifi_set_mode(WIFI_MODE_STA);
  btStart();
  WiFi.setSleep(false);
  delay(10);  // Wait for the modem to wake up
}

// sets the MCU sleep mode according to the operation mode
void esp_sleep(uint8_t mode, uint64_t *GPSInterval) {
  switch (mode) {
    case 0: // active mode
      *GPSInterval = GPS_interval_active; //set the location interval
      break;
    case 1:  // Parked mode
      {
        Serial.println("Entering light sleep");
        *GPSInterval = GPS_interval_parked; 
        setLED(0, true, false);
        esp_sleep_enable_timer_wakeup(GPS_interval_parked * 1000); //set an automatic wake-up timer to enable device to uplink
        esp_err_t err = esp_light_sleep_start(); // make it go to light sleep
        esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause(); //return the wake-up cause when waking up
        break;
      }
    case 2:
      {  // Storage mode
        *GPSInterval = GPS_interval_storage;
        Serial.println("Entering deep sleep");
        // turn off all connections
        Serial.end();
        setLED(0, true, false);
        // Write a '1' to the sleep bit in the config register https://www.analog.com/media/en/technical-documentation/data-sheets/max17048-max17049.pdf page 11
        uint8_t sleepBatteryConfig = 0x9C;
        Wire.beginTransmission(0x36);
        Wire.write(0x06);
        Wire.write(sleepBatteryConfig);
        Wire.endTransmission();
        //esp_sleep_enable_timer_wakeup(sleep_time_storage);
        esp_deep_sleep_start(); // go to sleep
        break;
      }
    default:
      break;
  }
}