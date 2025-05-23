#include "tracking.h"
#include "sleep.h"

uint8_t GPSTries = 0;
uint8_t WIFITries = 0;
uint64_t BSSID[20];

// Function to get location
void getPos(uint64_t *lastGPSTime, uint64_t *GPSInterval, SoftwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos, int8_t mode) {
  pos[0] = 0; // set default location to 0
  pos[1] = 0;
  
  if (millis() - *lastGPSTime >= *GPSInterval) {
    if (deviceState == DEVICE_STATE_SLEEP) {  //Check if ready to send again
      deviceState = DEVICE_STATE_SEND;
    } else { // if not ready to check, don't do more
      return;
    }

    *lastGPSTime = millis(); // set timer
    
    if (WIFITries < WIFI_max_tries) { // check if wifi-scan has failed more than 5 times, If it has, try GNSS
      WiFi.setSleep(false);
      uint8_t n = WIFI_scanning(pos); //make the wifi-scan
      if (n < 10) {  // If fix is not achived try again in 5 seconds. Threshold is set to be a minimum of 10 different MAC-addresses.
        Serial.println("No WIFI fix, trying again in 5 seconds.");
        *GPSInterval = GPS_interval_retry;  // 5 seconds
        WIFITries++;
      } else {  // If fix is achieved, go to sleep
        esp_sleep(mode, GPSInterval);
      }
      WiFi.setSleep(true);
    } else { // If wifi-scan failed, try GNSS
      GNSS(gpsSerial, gps, pos); // get location from GNSS
      if (pos[0] == 0.0 && pos[1] == 0.0) {  // If fix is not achived try again in 5 seconds
        Serial.println("No GPS fix, trying again in 5 seconds.");
        *GPSInterval = GPS_interval_retry;  // 5 seconds
        GPSTries++;
        if (GPSTries >= GPS_max_tries) {
          Serial.println("Max tries reached, going to sleep.");
          GPSTries = 0;
          WIFITries = 0;
          *GPSInterval = GPS_interval_timeout;  // 1 minute
          esp_sleep(mode, GPSInterval);
        }
      } else {          // If fix is achieved, go to sleep
        WIFITries = 0;  // Reset WIFI tries
        esp_sleep(mode, GPSInterval);
      }
    }
  }
}

// Gets the location from GNSS
bool GNSS(SoftwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos) {
  Serial.println("Get GPS fix");
  unsigned long start = millis();
  while (gpsSerial->available()) {
    gps->encode(gpsSerial->read()); //read the location from the module
  }

  if (gps->location.isValid()) {
    Serial.println("GPS success");
    float lat = gps->location.lat(); //get the latitude
    float lon = gps->location.lng(); //get the longitude
    pos[0] = lat;
    pos[1] = lon;
    return true;
  } else {
    Serial.println("GPS fail");
    pos[0] = 0.0;
    pos[1] = 0.0;
    return false;
  }
}

// gets MAC-addresses and RSSI, formats them for LoRa Transmission
uint8_t WIFI_scanning(float *pos) {
  // set location defualt to 0
  pos[0] = 0.0;
  pos[1] = 0.0;

  Serial.println("Scan start");
  int n = WiFi.scanNetworks(false, false, 1);  // scan networks, returns number of MACs found
  Serial.println("Scan done");
  for (int i = 0; i < 20; i++) {  //store up to 20 strongest WiFi-points found
    String BSSIDstr = WiFi.BSSIDstr(i);
    uint64_t RSSI = WiFi.RSSI(i);  //Byte 0 is the Signal strenght
    uint64_t byte1 = strtoul(BSSIDstr.substring(0, 2).c_str(), nullptr, 16); //the six following are the MAC adresss
    uint64_t byte2 = strtoul(BSSIDstr.substring(3, 5).c_str(), nullptr, 16);
    uint64_t byte3 = strtoul(BSSIDstr.substring(6, 8).c_str(), nullptr, 16);
    uint64_t byte4 = strtoul(BSSIDstr.substring(9, 11).c_str(), nullptr, 16);
    uint64_t byte5 = strtoul(BSSIDstr.substring(12, 14).c_str(), nullptr, 16);
    uint64_t byte6 = strtoul(BSSIDstr.substring(15, 17).c_str(), nullptr, 16);

    BSSID[i] = (RSSI << 48) | (byte1 << 40) | (byte2 << 32) | (byte3 << 24) | (byte4 << 16) | (byte5 << 8) | (byte6); //conncatenate into a single uint64_t
    BSSID[i] &= 0x00FFFFFFFFFFFFFF; //set Most significant byte to 0, as it is not used
  }

  return n;
}