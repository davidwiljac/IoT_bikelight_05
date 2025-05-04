#include "main.h"
#include "sleep.h"
#include "tracking.h"
#include "button.h"

// Pins
int LDRpin = 2;
int manualButtonpin = 3;
int INT1Pin = 0;
int RxPin = 1, TxPin = 1000;
int SDApin = 19, SCLpin = 18;
int LEDSer = 8, LEDClk = 9;

// Global variables
int8_t mode = 0;  // 0 = Active, 1 = Parked, 2 = Storage

Adafruit_MAX17048 batteryTracker;
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RxPin, TxPin);
//HardwareSerial gpsSerial(0);

bool buttonState = false;
bool showBattery = false;

uint64_t GPSInterval = 1000;  // 1 second
bool LEDstate = false;
bool lowLight = false;

int_config g_int_config_enabled = { 0 };
int_config g_int_config_map = { 0 };

static volatile bool accFlag = false;

float *pos = new float[2];  // Array to store the position
int8_t batteryPercent;
int8_t dischargeRate;

// Timer variables
uint64_t lastMoveTime = 0;
uint64_t lastOnTime = 0;
uint64_t lastGPSTime = 0;
uint64_t batteryIndicatorTime = 0;
// ---------------------

// const char *myssid = "HUAWEI P30 - Emil";  // your network SSID (name)
// const char *mypass = "HotSpot!36";         // your network password

// const char *myssid = "TP-Link_79DE";
// const char *mypass = "00895576";

const char *myssid = "Galaxy S20+ 5G ecb1";
const char *mypass = "KES12345";

// Credentials for Google GeoLocation API...
const char *Host = "www.googleapis.com";
String thisPage = "/geolocation/v1/geolocate?key=";
String key = "AIzaSyDo9og6Y61VADw3M3yrXWketfSZeoTMQTE";

String jsonString = "{\n";

int more_text = 1;  // set to 1 for more debug output

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xC8 };
uint8_t appEui[] = { 0xBE, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x16, 0x47 };
uint8_t appKey[] = { 0x38, 0xAA, 0xBB, 0x98, 0xFA, 0xE1, 0x7B, 0x7B, 0x1E, 0xDC, 0x81, 0x92, 0x18, 0xD6, 0x3A, 0xB1 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x2C, 0x47, 0x8D, 0x1D, 0x5B, 0x51, 0x53, 0x33, 0xEC, 0xC9, 0x62, 0x25, 0x32, 0x5B, 0xFF, 0xFC };
uint8_t appSKey[] = { 0x53, 0xE9, 0x6C, 0xD4, 0x58, 0x01, 0x58, 0x06, 0xB7, 0xB4, 0x1D, 0xEE, 0xD1, 0x43, 0x4F, 0xC1 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 10000;
uint32_t wifi_delay = 60000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
/*!
 * Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. The MAC performs a datarate adaptation,
 * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
 * to the following table:
 *
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 *
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
 */

uint8_t buffer = 0;
int last_time = 0;

uint8_t confirmedNbTrials = 4;

uint8_t received_data;
uint8_t data = 1;
uint8_t DataReceived[4];

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port, float lat, float lon) {
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
   *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
   *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
   *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
   *for example, if use REGION_CN470,
   *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
   */
  unsigned char *puc;

  appDataSize = 0;
  puc = (unsigned char *)(&LEDstate);
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&mode);
  appData[appDataSize++] = puc[0];

  Serial.println("Percent:" + String(batteryPercent) + "%");
  puc = (unsigned char *)(&batteryPercent);
  appData[appDataSize++] = puc[0];

  Serial.println("Discharge rate:" + String(dischargeRate) + "%/h");
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
}

// downlink data handle function example
void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    DataReceived[i] = mcpsIndication->Buffer[i];
    received_data = mcpsIndication->Buffer[i];
    Serial.print("received data: ");
    Serial.println(received_data);
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();

  if (received_data != 0) {
    data = received_data;
  }
  Serial.print("stored data: ");
  Serial.println(data);
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  Serial.println("Setup begin");

  // Set up the pins
  pinMode(LDRpin, INPUT);
  pinMode(manualButtonpin, INPUT);
  pinMode(INT1Pin, INPUT);
  pinMode(SCLpin, INPUT_PULLUP);
  pinMode(SDApin, INPUT_PULLUP);
  pinMode(LEDSer, OUTPUT);
  pinMode(LEDClk, OUTPUT);
  delay(1000);

  // initialize WIFI
  initWiFi();

  Wire.begin(SDApin, SCLpin);  // SDA, SCL
  Wire.setClock(100000);       // Set I2C clock speed to 100 kHz

  // Set up the battery tracker
  while (!batteryTracker.begin()) {
  }
  Serial.println("Found battery tracker!");

  // Set up the accelerometer
  if (!accel.begin(0x53)) {
    Serial.println("Failed to find ADXL343 chip");
  } else {
    Serial.println("Found ADXL343 chip!");
  }

  delay(100);
  accel.setRange(ADXL343_RANGE_2_G);
  attachInterrupt(digitalPinToInterrupt(INT1Pin), int1_isr, RISING);

  Wire.beginTransmission(0x53);
  Wire.write(0x31);  // Read data format register
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  uint8_t currenConfig = 0;
  if (Wire.available()) {
    currenConfig = Wire.read();
  }
  currenConfig &= 0xDF;  // Set accelerometer to active low

  Wire.beginTransmission(0x53);
  Wire.write(0x31);  // Write data format register
  Wire.write(currenConfig);
  Wire.endTransmission();

  g_int_config_enabled.bits.single_tap = true;
  accel.enableInterrupts(g_int_config_enabled);

  g_int_config_map.bits.single_tap = ADXL343_INT1;
  accel.mapInterrupts(g_int_config_map);

  accel.checkInterrupts();

  // // Set up the GPS
  gpsSerial.begin(GPSBaud);

  esp_deep_sleep_enable_gpio_wakeup((1ULL << INT1Pin) | (1ULL << manualButtonpin), ESP_GPIO_WAKEUP_GPIO_LOW);  // fra LoRaWANInterrupt example
  esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
  switch (wakeupReason) {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Woke up from timer sleep");
      mode = 2;
      break;
    case ESP_SLEEP_WAKEUP_GPIO:
      Serial.println("Woke up from light sleep");
      mode = 1;
      break;
  }
  Serial.println("Setup complete!");
}

void loop() {

  if (accFlag) {
    accFlag = false;
    accel.checkInterrupts();
  }

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

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
        digitalWrite(LEDSer, 1);  //Enable LORA¨
        delay(100);
        prepareTxFrame(appPort, pos[0], pos[1]);
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
        //LoRaWAN.sleep(loraWanClass);
        Mcu.timerhandler();
        Radio.IrqProcess();

        if (buffer == 0) {
          buffer = 1;
        }

        // if ((millis() - last_time) > txDutyCycleTime) {
        //   deviceState = DEVICE_STATE_SEND;
        //   last_time = millis();
        // }
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }

  switch (mode) {
    case 0:  // Active mode
      active();
      break;
    case 1:  // Parked mode
      park();
      break;
    case 2:
      storage();
      break;
    default:
      break;
  }
}

void int1_isr(void) {
  lastMoveTime = millis();
  accFlag = true;
}

uint64_t pritntTime = 0;
void active() {
  accel.checkInterrupts();

  // Reads battery status
  batteryPercent = int(batteryTracker.cellPercent());
  dischargeRate = int(batteryTracker.chargeRate());

  // Read the light sensor
  int light = analogRead(LDRpin);

  // Read the button and store the state in a list
  bool state = digitalRead(manualButtonpin);
  updateButtonStateList(state);

  uint8_t clickMode = readClickMode();
  if (clickMode == 1) {
    buttonState = !buttonState;
  } else if (clickMode == 2) {
    // Turn on battery indicator
    showBattery = true;
    batteryIndicatorTime = millis();
    setLED(batteryPercent, showBattery, LEDstate);
  } else if (clickMode == 3) {
    mode = 1;
    GPSInterval = GPS_interval_parked;
    lastMoveTime = millis();               // Reset the last move time
    lastGPSTime = millis() - GPSInterval;  // Force GPS to update

    LEDstate = false;
    buttonState = false;
    setLED(batteryPercent, showBattery, LEDstate);
    Serial.println("Switching to park mode due to manual turn off.");
    return;
  }

  if (showBattery && millis() - batteryIndicatorTime > 5000) {
    showBattery = false;
    setLED(batteryPercent, showBattery, LEDstate);
  }
  if (light < 1000) {
    if (!lowLight) {
      Serial.println("Low light detected!");
      lastOnTime = millis();
    }
    lowLight = true;
  } else {
    lowLight = false;
    lastOnTime = 0xFFFFFFFFFFFFF;
  }


  if (buttonState || ((millis() - lastOnTime > 5000) && lowLight)) {
    if (!LEDstate) {
      setLED(batteryPercent, showBattery, true);
    }
    LEDstate = true;
  } else {
    if (LEDstate) {
      setLED(batteryPercent, showBattery, false);
    }
    LEDstate = false;
  }


  if (millis() - lastMoveTime > swtich_to_park_time) {  // 120000
    mode = 1;
    GPSInterval = GPS_interval_parked;
    lastMoveTime = millis();               // Reset the last move time
    lastGPSTime = millis() - GPSInterval;  // Force GPS to update

    LEDstate = false;
    buttonState = false;
    setLED(batteryPercent, false, LEDstate);
    Serial.println("Switching to park mode due to inactivity.");
    return;
  }

  // Scans location every  minutes
  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);

  // modem_sleep(sleep_time_active); // Sleep for 1 second
}

void park() {
  if (accFlag) {
    accFlag = false;
    mode = 0;
    accel.checkInterrupts();
    GPSInterval = GPS_interval_active;
    Serial.println("Going back to active mode!");
  }

  if (millis() - lastMoveTime > swtich_to_storage_time) {
    mode = 2;                              // Switch to storage mode after 5 minutes of inactivity
    GPSInterval = GPS_interval_storage;    // 30 seconds
    lastMoveTime = millis();               // Reset the last move time
    lastGPSTime = millis() - GPSInterval;  // Force GPS to update
    Serial.println("Switching to storage mode due to inactivity.");
    return;
  }
  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}

void storage() {
  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}

//--------------------------------------------------------------------

void initWiFi() {
  // Set WiFi to station mode and disconnect from any previous AP
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Setup done");

  // // Connect to WiFi
  Serial.print("Connecting ");
  WiFi.begin(myssid, mypass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
}