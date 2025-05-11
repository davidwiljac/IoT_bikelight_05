#include "LoRa.h"

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port, bool LEDstate, int8_t mode, int8_t batteryPercent, int8_t dischargeRate, float lat, float lon, uint64_t GPS_interval_active, uint64_t GPS_interval_parked, uint64_t switch_to_park_time) {
  unsigned char *puc;

  appDataSize = 0;
  puc = (unsigned char *)(&LEDstate);
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&mode);
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&batteryPercent);
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&dischargeRate);
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&lat);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];

  puc = (unsigned char *)(&lon);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];

  uint16_t gps_active = GPS_interval_active / 1000 / 60;
  puc = (unsigned char *)(&gps_active);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];

  uint16_t gps_parked = GPS_interval_parked / 1000 / 60 / 60;
  puc = (unsigned char *)(&gps_parked);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];

  uint16_t switchPark = switch_to_park_time / 1000 / 10;
  puc = (unsigned char *)(&switchPark);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
}

// downlink data handle function example
void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    DataReceived[i] = mcpsIndication->Buffer[i];
    data = mcpsIndication->Buffer[i];
    Serial.print("received data: ");
    Serial.println(DataReceived[i]);
    Serial.println();
    Serial.printf("%02X", mcpsIndication->Buffer[i]);

    if ((data & 0b00111111) > 0) {
      if (((data & 0b11000000) >> 6) == 1) {
        GPS_interval_active = (data & 0b00111111) * 1000 * 60;  // minutes
      } else if (((data & 0b11000000) >> 6) == 2) {
        GPS_interval_parked = (data & 0b00111111) * 1000 * 60 * 60;  // hours
      } else if (((data & 0b11000000) >> 6) == 3) {
        switch_to_park_time = (data & 0b00111111) * 1000 * 10;  // 10s
      }
    }
    Serial.println("Active: " + String(GPS_interval_active / 1000 / 60) + "s");
    Serial.println("Parked: " + String(GPS_interval_parked / 1000 / 60 / 60) + "s");
    Serial.println("Switch to parked: " + String(switch_to_park_time / 1000 / 10) + "s");
  }
  Serial.println();
}

void LoRaLoop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
        LoRaWAN.init(loraWanClass, loraWanRegion);
        // both set join DR and DR when ADR off
        LoRaWAN.setDefaultDR(3);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        digitalWrite(LEDSer, 1);  //Enable LORA though the SPI slave select (Also used for LED serial comm)
        delay(100);
        prepareTxFrame(appPort, LEDstate, mode, batteryPercent, dischargeRate, pos[0], pos[1], GPS_interval_active, GPS_interval_parked, switch_to_park_time);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        buffer = 0;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);

        deviceState = DEVICE_STATE_SLEEP;
        buffer = 0;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        //Set the LoRa module to sleep mode to enable transmission
        Mcu.timerhandler();
        Radio.IrqProcess();

        if (buffer == 0) {
          buffer = 1;
        }
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}