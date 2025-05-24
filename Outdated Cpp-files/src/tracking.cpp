#include "tracking.h"
#include "sleep.h"

uint8_t GPSTries = 0;
uint8_t WIFITries = 0;

void getPos(uint64_t *lastGPSTime, uint64_t *GPSInterval, HardwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos, int8_t mode)
{
    if (millis() - *lastGPSTime >= *GPSInterval)
    {
        *lastGPSTime = millis();
        if (WIFITries < WIFI_max_tries)
        {
            WIFI_scanning(pos);
            if (pos[0] == 0.0 && pos[1] == 0.0)
            { // If fix is not achived try again in 5 seconds
                Serial.println("No WIFI fix, trying again in 5 seconds.");
                *GPSInterval = GPS_interval_active; // 5 seconds
                WIFITries++;
            }
            else
            { // If fix is achieved, go to sleep
                Serial.print("Latitude: ");
                Serial.println(pos[0], 6);
                Serial.print("Longitude: ");
                Serial.println(pos[1], 6);
                esp_sleep(mode, GPSInterval);
            }
        }
        else
        {
            GNSS(gpsSerial, gps, pos);
            if (pos[0] == 0.0 && pos[1] == 0.0)
            { // If fix is not achived try again in 5 seconds
                Serial.println("No GPS fix, trying again in 5 seconds.");
                *GPSInterval = GPS_interval_active; // 5 seconds
                GPSTries++;
                if (GPSTries >= GPS_max_tries)
                {
                    Serial.println("Max tries reached, going to sleep.");
                    GPSTries = 0;
                    *GPSInterval = GPS_interval_timeout; // 1 minute
                    esp_sleep(mode, GPSInterval);
                }
            }
            else
            { // If fix is achieved, go to sleep
                WIFITries = 0; // Reset WIFI tries
                Serial.print("Latitude: ");
                Serial.println(pos[0], 6);
                Serial.print("Longitude: ");
                Serial.println(pos[1], 6);
                esp_sleep(mode, GPSInterval);
            }
        }
    }
}

bool GNSS(HardwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos)
{
    unsigned long start = millis();
    while (gpsSerial->available())
    {
        gps->encode(gpsSerial->read());
    }

    if (gps->location.isValid())
    {
        float lat = gps->location.lat();
        float lon = gps->location.lng();
        pos[0] = lat;
        pos[1] = lon;
        return true;
    }
    else
    {
        pos[0] = 0.0;
        pos[1] = 0.0;
        return false;
    }
}

bool WIFI_scanning(float *pos)
{
    return true;
}