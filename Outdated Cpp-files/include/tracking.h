#include <TinyGPSPlus.h>
#include "main.h"
void getPos(uint64_t* lastGPSTime, uint64_t* GPSInterval, HardwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos, int8_t mode);
bool GNSS(HardwareSerial *gpsSerial, TinyGPSPlus *gps, float *pos);
bool WIFI_scanning(float* pos);