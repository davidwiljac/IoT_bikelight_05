#include "tracking.h"
#include "sleep.h"

uint8_t GPSTries = 0;
uint8_t WIFITries = 0;
uint64_t BSSID[20];

void getPos(uint64_t *lastGPSTime, uint64_t *GPSInterval, SoftwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos, int8_t mode) {
  pos[0] = 0;
  pos[1] = 0;
  if (millis() - *lastGPSTime >= *GPSInterval) {
    if (deviceState == DEVICE_STATE_SLEEP) { //Check if ready to send again
      deviceState = DEVICE_STATE_SEND;
    }else{
      return;
    }
    *lastGPSTime = millis();
    if (WIFITries < WIFI_max_tries) {
      WiFi.setSleep(false);
      uint8_t n = WIFI_scanning(pos);
      if (n < 10) {  // If fix is not achived try again in 5 seconds
        Serial.println("No WIFI fix, trying again in 5 seconds.");
        *GPSInterval = GPS_interval_retry;  // 5 seconds
        WIFITries++;
      } else {  // If fix is achieved, go to sleep
        esp_sleep(mode, GPSInterval);
      }
      WiFi.setSleep(true);
    } else {
      GNSS(gpsSerial, gps, pos);
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

bool GNSS(SoftwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos) {
  Serial.println("Get GPS fix");
  unsigned long start = millis();
  while (gpsSerial->available()) {
    gps->encode(gpsSerial->read());
  }

  if (gps->location.isValid()) {
    Serial.println("GPS success");
    float lat = gps->location.lat();
    float lon = gps->location.lng();
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

uint8_t WIFI_scanning(float *pos) {
  pos[0] = 0.0;
  pos[1] = 0.0;

  // Create a DynamicJsonDocument for parsing the response
  DynamicJsonDocument jsonBuffer(1024);

  Serial.println("Scan start");
  int n = WiFi.scanNetworks(false, false, 1);
  Serial.println("Scan done");
  for (int i = 0; i < 20; i++) {
    String BSSIDstr = WiFi.BSSIDstr(i);
    uint64_t RSSI = WiFi.RSSI(i);
    uint64_t byte1 = strtoul(BSSIDstr.substring(0, 2).c_str(), nullptr, 16);
    uint64_t byte2 = strtoul(BSSIDstr.substring(3, 5).c_str(), nullptr, 16);
    uint64_t byte3 = strtoul(BSSIDstr.substring(6, 8).c_str(), nullptr, 16);
    uint64_t byte4 = strtoul(BSSIDstr.substring(9, 11).c_str(), nullptr, 16);
    uint64_t byte5 = strtoul(BSSIDstr.substring(12, 14).c_str(), nullptr, 16);
    uint64_t byte6 = strtoul(BSSIDstr.substring(15, 17).c_str(), nullptr, 16);

    BSSID[i] = (RSSI << 48) | (byte1 << 40) | (byte2 << 32) | (byte3 << 24) | (byte4 << 16) | (byte5 << 8) | (byte6);
    BSSID[i] &= 0x00FFFFFFFFFFFFFF;
  }

  return n;
  // // Build the JSON string for the API request
  // jsonString = "{\n";
  // jsonString += "\"homeMobileCountryCode\": 234,\n";
  // jsonString += "\"homeMobileNetworkCode\": 27,\n";
  // jsonString += "\"radioType\": \"gsm\",\n";
  // jsonString += "\"carrier\": \"Vodafone\",\n";
  // jsonString += "\"wifiAccessPoints\": [\n";
  // for (int j = 0; j < n; ++j) {
  //   jsonString += "{\n";
  //   jsonString += "\"macAddress\" : \"";
  //   jsonString += WiFi.BSSIDstr(j);
  //   jsonString += "\",\n";
  //   jsonString += "\"signalStrength\": ";
  //   jsonString += WiFi.RSSI(j);
  //   jsonString += "\n";
  //   if (j < n - 1) {
  //     jsonString += "},\n";
  //   } else {
  //     jsonString += "}\n";
  //   }
  // }
  // jsonString += "]\n";
  // jsonString += "}\n";

  // // Needs actual root ca
  // WiFiClientSecure client;
  // client.setInsecure();  // Disable certificate verification (use with caution)

  // if (client.connect(Host, 443)) {
  //   Serial.println("Connected");
  //   client.println("POST " + thisPage + key + " HTTP/1.1");
  //   client.println("Host: " + String(Host));
  //   client.println("Connection: close");
  //   client.println("Content-Type: application/json");
  //   client.println("User-Agent: Arduino/1.0");
  //   client.print("Content-Length: ");
  //   client.println(jsonString.length());
  //   client.println();
  //   client.print(jsonString);
  //   delay(500);
  // }

  // // Read and parse the response from the server
  // while (client.available()) {
  //   String line = client.readStringUntil('\r');

  //   DeserializationError error = deserializeJson(jsonBuffer, line);
  //   if (!error) {
  //     if (jsonBuffer.containsKey("location")) {
  //       pos[0] = jsonBuffer["location"]["lat"];
  //       pos[1] = jsonBuffer["location"]["lng"];
  //     }
  //   }
  // }

  // client.stop();

  return true;
}