#include "button.h"
LinkedList<bool> buttonStateList = LinkedList<bool>();

bool button_has_been_released = true;
bool clickInit = false;
bool clickRelease = false;
bool wasClick = false;
uint64_t clickInitTime = 0xFFFFFFFFFFFFFFFF;
uint64_t clickReleaseTime = 0xFFFFFFFFFFFFFFFF;

void updateButtonStateList(bool state) {
  buttonStateList.add(state);
  if (buttonStateList.size() > SIZE_OF_BUTTON_ARRAY) {
    buttonStateList.remove(0);
  }
}

bool readButton() {
  bool state = false;
  if (buttonStateList.size() == SIZE_OF_BUTTON_ARRAY) {
    int count = 0;
    for (int i = 0; i < buttonStateList.size(); i++) {
      if (buttonStateList.get(i)) {
        count++;
      }
    }
    if (count == SIZE_OF_BUTTON_ARRAY) {
      state = true;
    } else {
      state = false;
    }
  }
  return state;
}

// Toggle button state if the button state has changed to high
bool toggleButtonState(bool buttonState) {
  if (readButton() && button_has_been_released) {
    button_has_been_released = false;
    buttonState = !buttonState;
  }
  if (!button_has_been_released && !readButton()) {
    button_has_been_released = true;
  }
  return buttonState;
}

/*

  returns clickState, 0 - no click, 1 - single click, 2 - double click, 3 - hold
*/
uint8_t readClickMode() {
  bool buttonRead = readButton();
  if (wasClick && buttonRead) {
    return 0;
  } else {
    wasClick = false;
  }

  if (!clickInit && buttonRead) {
    clickInit = true;
    clickInitTime = millis();
  } else if (clickInit && !buttonRead && !clickRelease) {
    clickRelease = true;
    clickReleaseTime = millis();
  }

  if (clickInit && !clickRelease && buttonRead && millis() - clickInitTime > 2000) {
    Serial.println("Hold");
    wasClick = true;
    clickInit = false;
    clickRelease = false;
    return 3;
  }
  if (clickInit && clickRelease && buttonRead && millis() - clickReleaseTime < 500) {
    Serial.println("Double click");
    wasClick = true;
    clickInit = false;
    clickRelease = false;
    return 2;
  }
  if (clickInit && clickRelease && millis() - clickReleaseTime > 500) {
    Serial.println("Single click");
    clickInit = false;
    clickRelease = false;
    return 1;
  }
  return 0;
}

void setLED(int8_t batteryPercent, bool showBattery, bool LEDstate) {
  bool* LEDArray = new bool[8];
  LEDArray[5] = LEDstate;
  LEDArray[6] = false;
  LEDArray[7] = false;

  float leveldec = batteryPercent / 20.0;
  uint8_t level = ceil(leveldec);
  if (!showBattery) {
    level = 0;
  }

  for (int i = 0; i < 5; i++) {
    if (i < level) {
      LEDArray[i] = true;
    } else {
      LEDArray[i] = false;
    }
  }
  digitalWrite(LEDSer, LOW);
  digitalWrite(LEDClk, LOW);
  delay(3);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(LEDSer, LEDArray[i]);
    delay(3);
    digitalWrite(LEDClk, HIGH);
    delay(3);
    digitalWrite(LEDClk, LOW);
    delay(3);
  }
  digitalWrite(LEDClk, HIGH);
  delay(3);
  digitalWrite(LEDClk, LOW);
  delay(3);
}
