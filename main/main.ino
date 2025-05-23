#include "main.h"

// Pins
#define LDRpin 2
#define manualButtonpin 3
#define INT1Pin 0   //accelerometer interrupt
#define RxPin 1     //receive pin for GNSS
#define TxPin 1000  // Not actually used, but has to be set to something
#define SDApin 19   // I2C SDA
#define SCLpin 18   // I2C SCL
#define LEDSer 8    // datapin for shift register
#define LEDClk 9    // clockpin for shift register
#define storageSwitch 20

// initialize Sensor modules
Adafruit_MAX17048 batteryTracker;
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);  // accelerometer

#define GPSBaud 9600
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RxPin, TxPin);  // software serial for UART connection to GNSS module.

// Global variables
int8_t mode = 0;  // 0 = Active, 1 = Parked, 2 = Storage

bool buttonState = false;
bool showBattery = false;

uint64_t GPSInterval = 1000;
bool LEDstate = false;
bool lowLight = false;
bool shouldBlink = false;
bool blinkStatus = false;
uint8_t numberOfBlinks = 0;

int_config g_int_config_enabled = { 0 };
int_config g_int_config_map = { 0 };

static volatile bool accFlag = false;

float *pos = new float[2];  // Array to store the position

int8_t batteryPercent;
int8_t dischargeRate;

// Timer variables
uint64_t lastMoveTime = 0;
uint64_t lightTime = 0;
uint64_t lowBatteryTime = 0;
uint64_t lowBatteryBlinkTime = 0;
uint64_t darknessTime = 0;
uint64_t lastGPSTime = 0;
uint64_t batteryIndicatorTime = 0;
uint16_t findSensorTime = 0xFFFF;
// ---------------------

// LoRa parameters
/* OTAA para*/  // device parameters for Cibicom
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
DeviceClass_t loraWanClass = CLASS_A;  // Class A is chosen for best battery life

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 10000;  // wait a minimum of approximatly 10s between sending

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;

// variables for storing received data
uint8_t data = 1;
uint8_t DataReceived[4];

void setup() {
  Serial.end();
  Serial.begin(115200, SERIAL_8N1, -1, 21);  // Start the serial connection to the computer. RX = -1 because it is not used after download.
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);    // initialize Heltec board
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

  // initialize WIFI
  initWiFi();
  btStop();

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

  // Write a 1 to the ENSLEEP bit to enable sleep mode for future use and send it. ref: https://www.analog.com/media/en/technical-documentation/data-sheets/max17048-max17049.pdf page 11
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

  // Set accelerometer to active high. See https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf page 27
  Wire.beginTransmission(0x53);
  Wire.write(0x31);  // Read data format register
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  uint8_t currenConfig = 0;
  if (Wire.available()) {
    currenConfig = Wire.read();
  }
  currenConfig &= 0xDF;

  Wire.beginTransmission(0x53);
  Wire.write(0x31);  // Write data format register
  Wire.write(currenConfig);
  Wire.endTransmission();

  // Set tap sensitivity to 1.6g. See https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf page 24
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
  // default wake-up/scan intervals in the modes
  GPS_interval_active = 30 * 1000;           // 120 seconds
  GPS_interval_parked = 8 * 60 * 60 * 1000;  // 8 hours
  switch_to_park_time = 120 * 1000;          // 30 seconds
  GPSInterval = GPS_interval_active;         // it starts in active mode

  gpsSerial.begin(GPSBaud);  // begin UART connection

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
      mode = 1;
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
  // check if device is set to storage mode by manual switch
  bool storageEnable = !digitalRead(storageSwitch);
  if (storageEnable && mode != 2) {
    mode = 2;
    Serial.println("Switching to storage mode!");
  }

  //Checks the LoRa state and acts accordingly
  LoRaLoop();

  //run code according to operation mode
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

// ----------------------------------------------------------------------------
void active() {
  // reset the accelerometer flag, if it has been set to active mode due to an accelerometer interrupt
  if (accFlag) {
    accFlag = false;
  }

  //Check the accelerometer
  accel.checkInterrupts();

  // Reads battery status
  batteryPercent = int(batteryTracker.cellPercent());
  //batteryPercent = 10; // Hardcoded to 10 in the demo-session, to show the low battery-warning
  dischargeRate = int(batteryTracker.chargeRate());


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
    setLED(batteryPercent, showBattery, LEDstate);  //turn of all of the LEDs
    Serial.println("Switching to park mode due to manual turn off.");
    return;
  }

  // If the battery indicators have been on for 5 seconds, turn off
  if (showBattery && millis() - batteryIndicatorTime > 5000) {
    showBattery = false;
    setLED(batteryPercent, showBattery, LEDstate);
  }

  
  // Read the light sensor
  int light = analogRead(LDRpin);

  // set the low light flag according to the LDR data
  if (light < 1000) {  // Start low light timer if below certain level, Threshold is 1000, in a range from 0-4047.
    if (!lowLight) {
      Serial.println("Low light detected!");
      darknessTime = millis();
    }
    lowLight = true;
    lightTime = 0xFFFFFFFFFFFFF;
  } else {
    if (lowLight) {
      Serial.println("High light detected!");
      lightTime = millis();
    }
    lowLight = false;
    darknessTime = 0xFFFFFFFFFFFFF;
  }

  // Low-battery warning, blinking the LEDs
  if (batteryPercent < 20) {
    if ((millis() - lowBatteryTime > 30000) && !shouldBlink) {
      shouldBlink = true;
      lowBatteryBlinkTime = millis();
    }
  }

  if (shouldBlink && (millis() - lowBatteryBlinkTime > 100)) {
    if (blinkStatus) {
      setLED(100, true, true);
    } else {
      setLED(0, true, false);
    }
    blinkStatus = !blinkStatus;
    lowBatteryBlinkTime = millis();
    if (numberOfBlinks > 6) {
      shouldBlink = false;
      lowBatteryTime = millis();
      numberOfBlinks = 0;
      setLED(batteryPercent, showBattery, LEDstate);
    } else {
      numberOfBlinks++;
    }
  }


  //Turn ON/Off the front and back lights accoring to, button-press aand light level
  if (buttonState || ((millis() - darknessTime > 5000) && lowLight)) {  // Turn LED on
    if (!LEDstate) {
      setLED(batteryPercent, showBattery, true);
    }
    LEDstate = true;
  } else if ((millis() - lightTime > 5000) && !lowLight) { //Turn LED Off
    if (LEDstate) {
      setLED(batteryPercent, showBattery, false);
    }
    LEDstate = false;
  }

  // check if mode should be set to Parked
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
  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}

void park() {
  if (accFlag) {  // If accelerometer pings, go to active
    accFlag = false;
    mode = 0; // go to active
    accel.checkInterrupts();
    GPSInterval = GPS_interval_active;
    Serial.println("Going back to active mode!");
  }

  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}

void storage() {
  esp_sleep(mode, &GPSInterval); // set mode to deep sleep
  //getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}

//--------------------------------------------------------------------

void initWiFi() {
  // Set WiFi to station mode and disconnect from any previous AP
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Setup done");

}