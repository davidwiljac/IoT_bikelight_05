static void prepareTxFrame(uint8_t port, bool LEDstate, int8_t mode, int8_t batteryPercent, int8_t dischargeRate, float lat, float lon);
void downLinkDataHandle(McpsIndication_t *mcpsIndication);
void LoRaLoop();