#include "button.h"
LinkedList<bool> buttonStateList = LinkedList<bool>();
bool button_has_been_released = true;
void updateButtonStateList(bool state)
{
    buttonStateList.add(state);
    if (buttonStateList.size() > SIZE_OF_BUTTON_ARRAY)
    {
        buttonStateList.remove(0);
    }
}

bool readButton()
{
    bool state = false;
    if (buttonStateList.size() == SIZE_OF_BUTTON_ARRAY)
    {
        int count = 0;
        for (int i = 0; i < buttonStateList.size(); i++)
        {
            if (buttonStateList.get(i))
            {
                count++;
            }
        }
        if (count == SIZE_OF_BUTTON_ARRAY)
        {
            state = true;
        }
        else
        {
            state = false;
        }
    }
    return state;
}

// Toggle button state if the button state has changed to high
bool toggleButtonState(bool buttonState)
{
    if (readButton() && button_has_been_released)
    {
        button_has_been_released = false;
        buttonState = !buttonState;
    }
    if (!button_has_been_released && !readButton())
    {
        button_has_been_released = true;
    }
    return buttonState;
}
