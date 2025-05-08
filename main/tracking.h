//#include <TinyGPSPlus.h>
#include "HT_TinyGPS++.h"

void getPos(uint64_t *lastGPSTime, uint64_t *GPSInterval, SoftwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos, int8_t mode);
bool GNSS(SoftwareSerial* gpsSerial, TinyGPSPlus* gps, float* pos);
bool WIFI_scanning(float* pos);