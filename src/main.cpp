#include "main.h"
#include "sleep.h"
#include "tracking.h"
#include "button.h"

// Pins
int LEDpin = 2;
int manualLEDpin = 32;
int LDRpin = 27;
int manualButtonpin = 34;
int INT1Pin = 33;
int RXPin = 16, TxPin = 17;
int SDAPin = 21, SCLPin = 22; // I2C pins
// Global variables
int8_t mode = 0; // 0 = Active, 1 = Parked, 2 = Storage

Adafruit_MAX17048 batteryTracker;
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

bool buttonState = false;

uint64_t lastOnTime = 0;
uint64_t lastGPSTime = 0;
uint64_t GPSInterval = 5000; // 5 second

bool LEDstate = false;
bool lowLight = false;

int_config g_int_config_enabled = { 0 };
int_config g_int_config_map = { 0 };

static volatile bool accFlag = false;

float* pos = new float[2]; // Array to store the position
// Timer variables
uint64_t lastMoveTime = 0;

void setup()
{
  // Set up the pins
  pinMode(LEDpin, OUTPUT);
  pinMode(LDRpin, INPUT);
  pinMode(manualButtonpin, INPUT);
  pinMode(manualLEDpin, OUTPUT);
  pinMode(INT1Pin, INPUT);
  delay(1000);
  Serial.begin(115200);
  
  Wire.begin(SDAPin, SCLPin); // SDA, SCL
  Wire.setClock(100000); // Set I2C clock speed to 100 kHz

  // Set up the battery tracker
  batteryTracker.begin();

  // Set up the accelerometer
  if(!accel.begin(0x53)){
    Serial.println("Failed to find ADXL343 chip");
  }

  delay(100);
  accel.setRange(ADXL343_RANGE_2_G);
  attachInterrupt(digitalPinToInterrupt(INT1Pin), int1_isr, RISING);

  Wire.beginTransmission(0x53);
  Wire.write(0x31); // Read data format register
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  uint8_t currenConfig = 0;
  if (Wire.available())
  {
    currenConfig = Wire.read();
  }
  currenConfig &= 0xDF; //Set accelerometer to active low

  Wire.beginTransmission(0x53);
  Wire.write(0x31); // Write data format register
  Wire.write(currenConfig);
  Wire.endTransmission();

  g_int_config_enabled.bits.single_tap = true;
  accel.enableInterrupts(g_int_config_enabled);

  g_int_config_map.bits.single_tap = ADXL343_INT1;
  accel.mapInterrupts(g_int_config_map);

  accel.checkInterrupts();

  // Set up the GPS¨
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TxPin);

  esp_sleep_enable_ext1_wakeup((1ULL << INT1Pin) | (1ULL << manualButtonpin), ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
  switch(wakeupReason){
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Woke up from timer sleep");
      mode = 2;
      break;
  }
  Serial.println("Setup complete!");
}

void loop()
{  
  if(accFlag){
    accFlag = false;
    accel.checkInterrupts();
  }

  switch (mode)
  {
  case 0: // Active mode
    active();
    break;
  case 1: // Parked mode
    park();
    break;
  case 2:
    storage();
    break;
  default:
    break;
  }
}

void int1_isr(void){
  lastMoveTime = millis();
  accFlag = true;
}

void active()
{
  accel.checkInterrupts();

  // Reads battery status
  /*
  float batteryPercent = batteryTracker.cellPercent();
  float dischargeRate = batteryTracker.chargeRate();
  */
  // Read the light sensor
  int light = analogRead(LDRpin);

  // Read the button and store the state in a list
  bool state = digitalRead(manualButtonpin);
  updateButtonStateList(state);
  buttonState = toggleButtonState(buttonState);
  if (light < 1000)
  {
    if (!lowLight)
    {
      Serial.println("Low light detected!");
      lastOnTime = millis();
    }
    lowLight = true;
  }
  else
  {
    lowLight = false;
    lastOnTime = 0xFFFFFFFFFFFFF;
  }

  if (buttonState || ((millis() - lastOnTime > 5000) && lowLight))
  {
    LEDstate = true;
  }
  else
  {
    LEDstate = false;
  }

  if(millis() - lastMoveTime > swtich_to_park_time){
    mode = 1;
    GPSInterval = GPS_interval_parked;
    lastMoveTime = millis(); // Reset the last move time
    lastGPSTime = millis() - GPSInterval; // Force GPS to update
    Serial.println("Switching to park mode due to inactivity.");
    return;
  }

  digitalWrite(manualLEDpin, buttonState);
  digitalWrite(LEDpin, LEDstate);
  
  GPSInterval = GPS_interval_active; // 5 seconds
  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);

  modem_sleep(sleep_time_active); // Sleep for 1 second
}

void park(){
  digitalWrite(manualLEDpin, LOW);
  digitalWrite(LEDpin, LOW);
  LEDstate = false;
  buttonState = false;

  if(accFlag){
    accFlag = false;
    mode = 0;
    accel.checkInterrupts();
    Serial.println("Going back to active mode!");
  }

  if(millis() - lastMoveTime > swtich_to_storage_time){
    mode = 2; // Switch to storage mode after 5 minutes of inactivity
    GPSInterval = GPS_interval_parked; // 30 seconds
    lastMoveTime = millis(); // Reset the last move time
    lastGPSTime = millis() - GPSInterval; // Force GPS to update
    Serial.println("Switching to storage mode due to inactivity.");
    return;
  }
  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}

void storage(){
  getPos(&lastGPSTime, &GPSInterval, &gpsSerial, &gps, pos, mode);
}