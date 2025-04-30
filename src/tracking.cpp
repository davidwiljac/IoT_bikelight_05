#include "tracking.h"
bool GNSS(HardwareSerial* gpsSerial, TinyGPSPlus* gps, float *pos)
{
    unsigned long start = millis();
     while (gpsSerial->available()){
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