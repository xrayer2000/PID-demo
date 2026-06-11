#include "TaskDisplay.h"

// Task handle definition
TaskHandle_t taskDisplayHandle = nullptr;

void TaskDisplay(void *pvParameters)
{
    while (true)
    {
        // TODO: OLED update

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}