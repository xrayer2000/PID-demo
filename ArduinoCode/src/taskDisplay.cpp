#include "main.h"
#include "taskDisplay.h"
#include "Axis/axis.h"

// Task handle definition
TaskHandle_t taskDisplayHandle = nullptr;

void taskDisplay(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (true)
    {
        // -----------------------------------------------------------------
        // Display sleep handling
        // -----------------------------------------------------------------

        float passedTime = millis() * 0.001f;
        float minutesSinceLastAction = (passedTime * 0.0166667f) - timeLastTouched;
        bool shouldSleep = (minutesSinceLastAction > settings.timeBeforeDisable);

        if (shouldSleep != displaySleeping)
        {
            displaySleeping = shouldSleep;
            display1.setPowerSave(displaySleeping);
            display2.setPowerSave(displaySleeping);
        }

        // -----------------------------------------------------------------
        // Draw current page
        // -----------------------------------------------------------------

        if (updateAllItems)
            display1.clearBuffer();

        switch (currPage)
        {
            case MENU_ROOT:          page_MenuRoot();         break;
            case MENU_AXIS_MODE:     page_MENU_AXIS_MODE(); break;
            case MENU_SELECTED_AXIS: page_MENU_SELECTED_AXIS(); break;
        }

        // -----------------------------------------------------------------
        // Push buffer to OLED
        // -----------------------------------------------------------------

        if (updateAllItems || updateItemValue)
        {
            display1.sendBuffer();
            updateAllItems = false;
            updateItemValue = false;
        }

        // -----------------------------------------------------------------
        // Update display 2
        // -----------------------------------------------------------------
        if (updateDisp2Flag)
        {
            updateDisp2();
            // display2.sendBuffer();   // send here, not inside updateDisp2
            // display2.clearBuffer();
            updateDisp2Flag = false;
        }

        // Run at 20 Hz
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_100HZ));
    }
}