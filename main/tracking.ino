#include "tracking.h"
#include "sleep.h"

uint8_t GPSTries = 0;
uint8_t WIFITries = 0;

void getPos(uint64_t *lastGPSTime, uint64_t *GPSInterval, SoftwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos, int8_t mode) {
  if (millis() - *lastGPSTime >= *GPSInterval) {
    deviceState = DEVICE_STATE_SEND;
    *lastGPSTime = millis();
    if (WIFITries < WIFI_max_tries) {
      WiFi.setSleep(false);
      WIFI_scanning(pos);
      if (pos[0] == 0.0 && pos[1] == 0.0) {  // If fix is not achived try again in 5 seconds
        Serial.println("No WIFI fix, trying again in 5 seconds.");
        *GPSInterval = GPS_interval_active;  // 5 seconds
        WIFITries++;
      } else {  // If fix is achieved, go to sleep
        Serial.print("Latitude: ");
        Serial.println(pos[0], 6);
        Serial.print("Longitude: ");
        Serial.println(pos[1], 6);
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
          *GPSInterval = GPS_interval_timeout;  // 1 minute
          esp_sleep(mode, GPSInterval);
        }
      } else {          // If fix is achieved, go to sleep
        WIFITries = 0;  // Reset WIFI tries
        Serial.print("Latitude: ");
        Serial.println(pos[0], 6);
        Serial.print("Longitude: ");
        Serial.println(pos[1], 6);
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

bool WIFI_scanning(float *pos) {
  pos[0] = 0.0;
  pos[1] = 0.0;

  // Create a DynamicJsonDocument for parsing the response
  DynamicJsonDocument jsonBuffer(1024);

  Serial.println("Scan start");
  int n = WiFi.scanNetworks(false, false, 1);
  Serial.println("Scan done");

  // Build the JSON string for the API request
  jsonString = "{\n";
  jsonString += "\"homeMobileCountryCode\": 234,\n";
  jsonString += "\"homeMobileNetworkCode\": 27,\n";
  jsonString += "\"radioType\": \"gsm\",\n";
  jsonString += "\"carrier\": \"Vodafone\",\n";
  jsonString += "\"wifiAccessPoints\": [\n";
  for (int j = 0; j < n; ++j) {
    jsonString += "{\n";
    jsonString += "\"macAddress\" : \"";
    jsonString += WiFi.BSSIDstr(j);
    jsonString += "\",\n";
    jsonString += "\"signalStrength\": ";
    jsonString += WiFi.RSSI(j);
    jsonString += "\n";
    if (j < n - 1) {
      jsonString += "},\n";
    } else {
      jsonString += "}\n";
    }
  }
  jsonString += "]\n";
  jsonString += "}\n";

  // Needs actual root ca
  WiFiClientSecure client;
  client.setInsecure();  // Disable certificate verification (use with caution)

  if (client.connect(Host, 443)) {
    Serial.println("Connected");
    client.println("POST " + thisPage + key + " HTTP/1.1");
    client.println("Host: " + String(Host));
    client.println("Connection: close");
    client.println("Content-Type: application/json");
    client.println("User-Agent: Arduino/1.0");
    client.print("Content-Length: ");
    client.println(jsonString.length());
    client.println();
    client.print(jsonString);
    delay(500);
  }

  // Read and parse the response from the server
  while (client.available()) {
    String line = client.readStringUntil('\r');

    DeserializationError error = deserializeJson(jsonBuffer, line);
    if (!error) {
      if (jsonBuffer.containsKey("location")) {
        pos[0] = jsonBuffer["location"]["lat"];
        pos[1] = jsonBuffer["location"]["lng"];
      }
    }
  }

  client.stop();

  return true;
}