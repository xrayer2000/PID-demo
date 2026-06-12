#include "main.h"
#include "taskDisplay.h"

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

        switch (currPage)
        {
            case MENU_ROOT:        page_MenuRoot();         break;
            case MENU_SYSTEM_MODE: page_MENU_SYSTEM_MODE(); break;
        }

        // -----------------------------------------------------------------
        // Push buffer to OLED
        // -----------------------------------------------------------------

        if (updateAllItems || updateItemValue)
        {
            display1.sendBuffer();
            display2.sendBuffer();

            updateAllItems = false;
            updateItemValue = false;
        }

        // Run at 20 Hz
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_50HZ));
    }
}