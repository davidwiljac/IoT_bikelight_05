#include "sleep.h"
#include "main.h"
void modem_sleep(int32_t time_millis)
{
  WiFi.setSleep(true);
  btStop();
  esp_wifi_set_mode(WIFI_MODE_NULL);
}

void modem_wake()
{
  esp_wifi_set_mode(WIFI_MODE_STA);
  btStart();
  WiFi.setSleep(false);
  delay(10); // Wait for the modem to wake up
}

void esp_sleep(uint8_t mode, uint64_t *GPSInterval)
{
  switch (mode)
  {
  case 0:                               // Active mode7
    *GPSInterval = GPS_interval_active; // 5 seconds
    break;
  case 1:                                           // Parked mode
    *GPSInterval = GPS_interval_parked;             // 2 minutes
    esp_sleep_enable_timer_wakeup(sleep_time_park); // Sleep for 5 minutes
    esp_light_sleep_start();
    break;
  case 2:                                              // Storage mode
    *GPSInterval = GPS_interval_storage;               // 30 seconds
    esp_sleep_enable_timer_wakeup(sleep_time_storage); // Sleep for 30 seconds
    esp_deep_sleep_start();
    break;
  default:
    break;
  }
}