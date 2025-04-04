#include <Arduino.h>
#include <LinkedList.h>
#include "Adafruit_MAX1704X.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL343.h"

bool readButton();
bool toggleButtonState(bool buttonState, LinkedList<bool> buttonStateList, bool button_has_been_released);
void active();
void int1_isr(void);