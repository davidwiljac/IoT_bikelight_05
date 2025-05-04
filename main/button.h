#include <LinkedList.h>

#define SIZE_OF_BUTTON_ARRAY 3



void updateButtonStateList(bool state);
bool readButton();
bool toggleButtonState(bool buttonState);
uint8_t readClickMode();
void setLED(int8_t batteryPercent, bool showBattery, bool LEDstate);
