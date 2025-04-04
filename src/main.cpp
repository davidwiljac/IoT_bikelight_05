#include "main.h"
#include "sleep.h"
#include "tracking.h"
#include "button.h"

// Pins
int LEDpin = 2;
int manualLEDpin = 32;
int LDRpin = 27;
int manualButtonpin = 34;
int AccelPin = 35;
int INT1Pin = 33;

// Global variables
int8_t mode = 0; // 0 = Active, 1 = Parked, 2 = Storage

Adafruit_MAX17048 batteryTracker;
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

bool buttonState = false;

uint64_t lastOnTime = 0;
bool LEDstate = false;
bool lowLight = false;

int_config g_int_config_enabled = { 0 };
int_config g_int_config_map = { 0 };

// Timer variables
hw_timer_t *Timer0_Cfg = NULL;

void setup()
{
  // Set up the pins
  pinMode(LEDpin, OUTPUT);
  pinMode(LDRpin, INPUT);
  pinMode(manualButtonpin, INPUT);
  pinMode(manualLEDpin, OUTPUT);
  pinMode(AccelPin, INPUT);
  pinMode(INT1Pin, INPUT);

  // Set up the battery tracker
  batteryTracker.begin();

  // Set up the accelerometer
  accel.begin();
  accel.setRange(ADXL343_RANGE_2_G);
  attachInterrupt(digitalPinToInterrupt(INT1Pin), int1_isr, CHANGE);
  g_int_config_enabled.bits.single_tap = true;
  accel.enableInterrupts(g_int_config_enabled);

  g_int_config_map.bits.single_tap = ADXL343_INT1;
  accel.mapInterrupts(g_int_config_map);

  accel.checkInterrupts();

  Serial.begin(115200);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)manualButtonpin, HIGH); // Wake up on button press

  Serial.println("");
}

void loop()
{  
  accel.checkInterrupts();
  switch (mode)
  {
  case 0: // Active mode
    active();
    break;

  default:
    break;
  }
}

void int1_isr(void){
  Serial.println("Wake up!");
}

void active()
{
  // Reads battery status
  float batteryPercent = batteryTracker.cellPercent();
  float dischargeRate = batteryTracker.chargeRate();

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

  digitalWrite(manualLEDpin, buttonState);
  digitalWrite(LEDpin, LEDstate);

  modem_sleep(1000); // Sleep for 1 second
}