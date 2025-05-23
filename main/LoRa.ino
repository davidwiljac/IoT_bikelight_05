#include "LoRa.h"
// get global variables from other files
extern uint64_t BSSID[20];
extern uint64_t GPS_interval_active;
extern uint64_t GPS_interval_parked;
extern uint64_t switch_to_park_time;

// Prepares the payload that is to be uplinked
static void prepareTxFrame(uint8_t port, bool LEDstate, int8_t mode, int8_t batteryPercent, int8_t dischargeRate, float lat, float lon) {
  unsigned char *puc;

  appDataSize = 0;
  puc = (unsigned char *)(&LEDstate);  // set byte 1 to wheter the light is on or off
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&mode);  // set byte 2 to the current operation mode
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&batteryPercent);  // set byte 3 to the battery percentage
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&dischargeRate);  // set byte 4 to the current discharge rate
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&lat);  // byte 5-8, is the latitude from GNSS
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];

  puc = (unsigned char *)(&lon);  // byte 9-12 is the longitude from GNSS
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];

  uint16_t gps_active = GPS_interval_active / 1000 / 60;  // concatenate wake-up interval in active mode to minutes
  puc = (unsigned char *)(&gps_active);                   // byte 13-14 is active interval
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  uint16_t gps_parked = GPS_interval_parked / 1000 / 60 / 60;  // concatenate wake-up interval in parked mode to hours
  puc = (unsigned char *)(&gps_parked);                        // byte 15-16 is parked interval
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  uint16_t switchPark = switch_to_park_time / 1000 / 10;  // concatenate the switch to park mode-interval to intervals of 10s
  puc = (unsigned char *)(&switchPark);                   // byte 17-18 is switch interval
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  for (int i = 0; i < 20; i++) {  // byte 19-158 is the 20 MAC addresses
    puc = (unsigned char *)(&BSSID[i]);
    for (int j = 6; j >= 0; j--) {
      appData[appDataSize++] = puc[j];
    }
  }
}

// downlink data handle function. Runs when data has been received
void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  // print the data, signal strenghts and more
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    DataReceived[i] = mcpsIndication->Buffer[i];
    data = mcpsIndication->Buffer[i];
    Serial.print("received data: ");
    Serial.println(DataReceived[i]);
    Serial.println();
    Serial.printf("%02X", mcpsIndication->Buffer[i]);

    if ((data & 0b00111111) > 0) { // check if data is different from 0. The intervals can not be set to 0
      // Two MSB of received data indicates which mode is trying to be set 1: active, 2: parked, 3: switch 
      if (((data & 0b11000000) >> 6) == 1) { 
        GPS_interval_active = (data & 0b00111111) * 1000 * 60;  // minutes, Update interval to number in minutes
      } else if (((data & 0b11000000) >> 6) == 2) {
        GPS_interval_parked = (data & 0b00111111) * 1000 * 60 * 60;  // hours, uppdate interval to number in seconds
      } else if (((data & 0b11000000) >> 6) == 3) {
        switch_to_park_time = (data & 0b00111111) * 1000 * 10;  // 10s, update interval to number in intervals of 10s
      }
    }
    Serial.println("Active: " + String(GPS_interval_active / 1000 / 60) + "s");
    Serial.println("Parked: " + String(GPS_interval_parked / 1000 / 60 / 60) + "s");
    Serial.println("Switch to parked: " + String(switch_to_park_time / 1000 / 10) + "s");
  }
  Serial.println();
}

// state machine for handling the LoRa cycle
void LoRaLoop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT: // initialize LoRa, if it has not been done before
      {
        LoRaWAN.init(loraWanClass, loraWanRegion);
        LoRaWAN.setDefaultDR(4); // set default data rate to 4.
        //LoRaWAN.setDataRateForNoADR(4);
        break;
      }
    case DEVICE_STATE_JOIN: // establish connection to Cibicom.
      {
        LoRaWAN.join();
        Serial.println("Join done!");
        break;
      }
    case DEVICE_STATE_SEND:
      {
        Serial.println("Sending!");
        digitalWrite(LEDSer, 1);  //Enable LoRa-module through the SPI slave select (Also used for LED serial comm)
        delay(100);
        prepareTxFrame(appPort, LEDstate, mode, batteryPercent, dischargeRate, pos[0], pos[1]);
        LoRaWAN.send(); // transmit the data
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND); // add random interval to avoid constant duty cycle and overload the network
        LoRaWAN.cycle(txDutyCycleTime);

        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        //Set the LoRa module to sleep mode to enable transmission
        Mcu.timerhandler();
        Radio.IrqProcess();
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}