static void prepareTxFrame(uint8_t port, bool LEDstate, int8_t mode, int8_t batteryPercent, int8_t dischargeRate, float lat, float lon, uint64_t GPS_interval_active, uint64_t GPS_interval_parked, uint64_t switch_to_park_time);
void downLinkDataHandle(McpsIndication_t *mcpsIndication);
void LoRaLoop();