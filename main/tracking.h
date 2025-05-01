//#include <TinyGPSPlus.h>
#include "HT_TinyGPS++.h"
bool GNSS(HardwareSerial* gpsSerial, TinyGPSPlus* gps, float* pos);
bool WIFI_scanning(float* pos);