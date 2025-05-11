#include "main.h"

// Pins
#define LDRpin 2
#define manualButtonpin 3
#define INT1Pin 0
#define RxPin 1
#define TxPin 1000  // Not actually used, but has to be set to something
#define SDApin 19
#define SCLpin 18
#define LEDSer 8
#define LEDClk 9
#define storageSwitch 20

// Global variables
int8_t mode = 0;  // 0 = Active, 1 = Parked, 2 = Storage

Adafruit_MAX17048 batteryTracker;
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

#define GPSBaud 9600
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RxPin, TxPin);

bool buttonState = false;
bool showBattery = false;

uint64_t GPSInterval = 1000;
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
uint16_t findSensorTime = 0xFFFF;
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

// LoRa parameters
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

uint8_t buffer = 0;
int last_time = 0;

uint8_t confirmedNbTrials = 4;

uint8_t received_data;
uint8_t data = 1;
uint8_t DataReceived[4];

void setup() {
  Serial.end();
  Serial.begin(115200, SERIAL_8N1, -1, 21);  // Example: use GPIO9 (RX), GPIO10 (TX)
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  Serial.println("Setup begin");

  // Set up the pins
  pinMode(LDRpin, INPUT);
  pinMode(manualButtonpin, INPUT_PULLDOWN);
  pinMode(INT1Pin, INPUT_PULLDOWN);
  pinMode(SCLpin, INPUT_PULLUP);
  pinMode(SDApin, INPUT_PULLUP);
  pinMode(LEDSer, OUTPUT);
  pinMode(LEDClk, OUTPUT);
  pinMode(storageSwitch, INPUT);

  // Reset LED
  setLED(0, false, false);
  pinMode(2, OUTPUT);

  // initialize WIFI
  //initWiFi();

  Wire.begin(SDApin, SCLpin);  // SDA, SCL
  Wire.setClock(100000);       // Set I2C clock speed to 100 kHz

  // Set up the battery tracker
  findSensorTime = millis();
  while (!batteryTracker.begin()) {
    if (millis() - findSensorTime > 5000) {
      Serial.println("Failed to find battery tracker");
      break;
    }
  }
  if (millis() - findSensorTime < 5000) {
    Serial.println("Found battery tracker");
  }
  // Read the current mode of the batterytracker
  Wire.beginTransmission(0x36);
  Wire.write(0x06);  // Read data format register
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  uint8_t currentMode = 0;
  if (Wire.available()) {
    currentMode = Wire.read();
  }

  // Write a 1 to the ENSLEEP bit to enable sleep mode for future use and send it https://www.analog.com/media/en/technical-documentation/data-sheets/max17048-max17049.pdf page 11
  currentMode &= B00100000;
  Wire.beginTransmission(0x36);
  Wire.write(0x06);  // Write data format register
  Wire.write(currentMode);
  Wire.endTransmission();

  // Ensures the batterytracker is in standard mode. Same place as previous block.
  uint8_t defaultBatteryConfig = 0x1C;
  Wire.beginTransmission(0x36);
  Wire.write(0x06);
  Wire.write(defaultBatteryConfig);
  Wire.endTransmission();

  // Set up the accelerometer
  findSensorTime = millis();
  while (!accel.begin(0x53)) {
    if (millis() - findSensorTime > 5000) {
      Serial.println("Failed to find ADXL353");
      break;
    }
  }
  if (millis() - findSensorTime < 5000) {
    Serial.println("Found ADXL353");
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
  // Set accelerometer to active high. See https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf page 27
  currenConfig &= 0xDF;

  Wire.beginTransmission(0x53);
  Wire.write(0x31);  // Write data format register
  Wire.write(currenConfig);
  Wire.endTransmission();

  // Set tap sensitivity to 2g. See https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf page 24
  Wire.beginTransmission(0x53);
  Wire.write(0x1D);
  Wire.write(32);
  Wire.endTransmission();

  // Set to low power mode. See https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf page 25
  Wire.beginTransmission(0x53);
  Wire.write(0x2C);
  Wire.write(0x17);  // Low power bit and 0111 (6.25Hz) for bandwith
  Wire.endTransmission();


  // Set the INT1 pin to fire on a single tap
  g_int_config_enabled.bits.single_tap = true;
  accel.enableInterrupts(g_int_config_enabled);

  g_int_config_map.bits.single_tap = ADXL343_INT1;
  accel.mapInterrupts(g_int_config_map);

  accel.checkInterrupts();


  // Set up the GPS
  GPS_interval_active = 30 * 1000;    // 30 seconds
  GPS_interval_parked = 30 * 1000;    // 2 minutes
  switch_to_park_time = 1000 * 1000;  // 60 seconds
  GPSInterval = GPS_interval_active;

  gpsSerial.begin(GPSBaud);

  // Enable wakeups from button and acc intterupt
  uint64_t mask = 0;
  mask |= (1ULL << INT1Pin);
  mask |= (1ULL << manualButtonpin);
  esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_HIGH);  // fra LoRaWANInterrupt example

  esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
  Serial.println("reason: " + wakeupReason);
  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_GPIO:
      printf("Wakeup caused by GPIO\n");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      printf("Wakeup caused by timer\n");
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      printf("Wakeup cause: undefined\n");
      break;
    default:
      printf("Other wakeup cause\n");
      break;
  }

  Serial.println("Setup complete!");
}

void loop() {
  bool storageEnable = !digitalRead(storageSwitch);
  if (storageEnable && mode != 2) {
    mode = 2;
    Serial.println("Switching to storage mode!");
  }
  //Checks the LoRa state and acts accordingly
  LoRaLoop();

  switch (mode) {
    case 0:  // Active mode
      active();
      break;
    case 1:  // Parked mode
      park();
      break;
    case 2:  // Storage mode
      storage();
      break;
    default:
      break;
  }
}

// Accelerometer interupt
void int1_isr(void) {
  lastMoveTime = millis();
  accFlag = true;
}

void active() {
  if (accFlag) {
    accFlag = false;
  }

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
  if (clickMode == 1) {  // Single click, toggle the light
    buttonState = !buttonState;
  } else if (clickMode == 2) {  // Turn on battery indicator
    showBattery = true;
    batteryIndicatorTime = millis();
    setLED(batteryPercent, showBattery, LEDstate);
  } else if (clickMode == 3) {  // Go to park mode !!TODO: Should go to storage mode??? !!
    mode = 1;
    GPSInterval = GPS_interval_parked;
    lastMoveTime = millis();               // Reset the last move time
    lastGPSTime = millis() - GPSInterval;  // Force GPS to update

    LEDstate = false;
    buttonState = false;
    accFlag = false;
    setLED(batteryPercent, showBattery, LEDstate);
    Serial.println("Switching to park mode due to manual turn off.");
    return;
  }

  if (showBattery && millis() - batteryIndicatorTime > 5000) {  // If the battery indicators have been on for 5 seconds, turn off
    showBattery = false;
    setLED(batteryPercent, showBattery, LEDstate);
  }

  if (light < 1000) {  // Start low light timer if below certain level
    if (!lowLight) {
      Serial.println("Low light detected!");
      lastOnTime = millis();
    }
    lowLight = true;
  } else {
    lowLight = false;
    lastOnTime = 0xFFFFFFFFFFFFF;
  }


  if (buttonState || ((millis() - lastOnTime > 5000) && lowLight)) {  // Turn LED on/off
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


  if (millis() - lastMoveTime > switch_to_park_time) {  // Switch to park mode after an amout of time has passed
    mode = 1;
    GPSInterval = GPS_interval_parked;
    lastMoveTime = millis();               // Reset the last move time
    lastGPSTime = millis() - GPSInterval;  // Force GPS to update

    LEDstate = false;
    buttonState = false;
    accFlag = false;
    setLED(batteryPercent, false, LEDstate);
    Serial.println("Switching to park mode due to inactivity.");
    return;
  }

  // Scans location if appropriate time has passed
  //getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}

void park() {
  if (accFlag) {  // If accelerometer pings, go to active
    accFlag = false;
    mode = 0;
    accel.checkInterrupts();
    GPSInterval = GPS_interval_active;
    Serial.println("Going back to active mode!");
  }

  // if (millis() - lastMoveTime > switch_to_storage_time) {
  //   mode = 2;  // Switch to storage mode after a period of inactivity
  //   GPSInterval = GPS_interval_storage;
  //   lastMoveTime = millis();               // Reset the last move time
  //   lastGPSTime = millis() - GPSInterval;  // Force GPS to update
  //   Serial.println("Switching to storage mode due to inactivity.");
  //   return;
  // }
  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}

void storage() {
  esp_sleep(mode, &GPSInterval);
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