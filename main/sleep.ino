#include "sleep.h"
void modem_sleep(int32_t time_millis) {
  WiFi.setSleep(true);
  btStop();
  esp_wifi_set_mode(WIFI_MODE_NULL);
}

void modem_wake() {
  esp_wifi_set_mode(WIFI_MODE_STA);
  btStart();
  WiFi.setSleep(false);
  delay(10);  // Wait for the modem to wake up
}

void esp_sleep(uint8_t mode, uint64_t *GPSInterval) {
  switch (mode) {
    case 0:                                
      *GPSInterval = GPS_interval_active;
      break;
    case 1:  // Parked mode
      Serial.println("Entering light sleep");
      *GPSInterval = GPS_interval_parked;
      esp_sleep_enable_timer_wakeup(sleep_time_park);
      esp_light_sleep_start();
      break;
    case 2:  // Storage mode
      Serial.println("Entering deep sleep");
      *GPSInterval = GPS_interval_storage;
      esp_sleep_enable_timer_wakeup(sleep_time_storage);
      esp_deep_sleep_start();
      break;
    default:
      break;
  }
}