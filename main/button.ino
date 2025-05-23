#include "button.h"
LinkedList<bool> buttonStateList = LinkedList<bool>(); //linked list to trace the button value

// global variables
bool button_has_been_released = true;
bool clickInit = false;
bool clickRelease = false;
bool wasClick = false;
uint64_t clickInitTime = 0xFFFFFFFFFFFFFFFF;
uint64_t clickReleaseTime = 0xFFFFFFFFFFFFFFFF;

void updateButtonStateList(bool state) { // add the current button state to the linked list
  buttonStateList.add(state);
  if (buttonStateList.size() > SIZE_OF_BUTTON_ARRAY) { // remove first element, if list size has been exceeded
    buttonStateList.remove(0);
  }
}

// debounceing function for the button
bool readButton() {
  bool state = false;
  if (buttonStateList.size() == SIZE_OF_BUTTON_ARRAY) { //check if linked list is fully initialized
    int count = 0;
    for (int i = 0; i < buttonStateList.size(); i++) { //count the amount of 1's (pressed) in list
      if (buttonStateList.get(i)) {
        count++;
      }
    }
    if (count == SIZE_OF_BUTTON_ARRAY) { // only if the list is full of 1's the button is pressed
      state = true;
    } else { // not pressed
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


// Function to detect click type
uint8_t readClickMode() {
  // returns clickState, 0 - no click, 1 - single click, 2 - double click, 3 - hold
  bool buttonRead = readButton();
  if (wasClick && buttonRead) { // if a click already has been detected, do not register more clicks 
    return 0;
  } else {
    wasClick = false;
  }

  //detect the begining of a click
  if (!clickInit && buttonRead) { 
    clickInit = true;
    clickInitTime = millis();
  } else if (clickInit && !buttonRead && !clickRelease) { //detects the end of a click
    clickRelease = true;
    clickReleaseTime = millis();
  }

  // hold
  // if a click has been started and it is still there after 2s, it has ben held down
  if (clickInit && !clickRelease && buttonRead && millis() - clickInitTime > 2000) {
    Serial.println("Hold");
    wasClick = true;
    clickInit = false;
    clickRelease = false;
    return 3;
  }

  // double click
  // if a click has been both started and stoped and started again within 0.5s it is a double click.
  if (clickInit && clickRelease && buttonRead && millis() - clickReleaseTime < 500) {
    Serial.println("Double click");
    wasClick = true;
    clickInit = false;
    clickRelease = false;
    return 2;
  }

  // single click
  // if a click has been started and stopped, and a double click have not been detected, it is a single click
  if (clickInit && clickRelease && millis() - clickReleaseTime > 500) {
    Serial.println("Single click");
    clickInit = false;
    clickRelease = false;
    return 1;
  }
  return 0; // otherwise no click
}

// Turns the battery indicator LEDs and bike lights on/off accordingly
void setLED(int8_t batteryPercent, bool showBattery, bool LEDstate) {
  bool* LEDArray = new bool[8]; // array on the size of the shiftregister
  LEDArray[5] = LEDstate; //set output 5 to be the bike lights
  LEDArray[6] = false; //output pin of shiftregister not used
  LEDArray[7] = false;

  // determine how many LEDs should be turned on
  float leveldec = batteryPercent / 20.0; // 5xLEDs => intervals of 20%
  uint8_t level = ceil(leveldec);
  if (!showBattery) { // turn them off if not to be turned on.
    level = 0;
  }

  for (int i = 0; i < 5; i++) { // turn on according to level
    if (i < level) {
      LEDArray[i] = true;
    } else {
      LEDArray[i] = false;
    }
  }
  
  // transmit the data to the shift register
  digitalWrite(LEDSer, LOW);
  digitalWrite(LEDClk, LOW);
  delay(1);
  for (int i = 7; i >= 0; i--) { // shift the values into the register. Wait 1ms to ensure data validity
    digitalWrite(LEDSer, LEDArray[i]); // set data
    delay(1);
    digitalWrite(LEDClk, HIGH); //set clock high
    delay(1);
    digitalWrite(LEDClk, LOW); //set clock low
    delay(1);
  }
  digitalWrite(LEDClk, HIGH); // pass all data through second layer of the shift register.
  delay(1);
  digitalWrite(LEDClk, LOW);
  delay(1);
}
