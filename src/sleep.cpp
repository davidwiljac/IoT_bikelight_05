#include "sleep.h"

void modem_sleep(int32_t time_millis)
{
  WiFi.mode(WIFI_OFF);
  btStop();
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_sleep_enable_timer_wakeup(time_millis * 1000); // Wake up after 1 second
  esp_light_sleep_start();
}