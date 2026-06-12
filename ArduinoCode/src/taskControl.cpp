#include "main.h"
#include "taskControl.h"
#include "Axis/axis.h"

// Task handle definition
TaskHandle_t taskControlHandle = nullptr;

void taskControl(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        float passedTime = millis() * 0.001f;
        const float UPDATE_INTERVAL1 = 0.05f;
        bool shouldUpdate1 = (passedTime - previousPassedTime1 >= UPDATE_INTERVAL1);

        updateAxis(axis, pid);

        if (shouldUpdate1) {
            uint32_t ms       = millis();
            uint32_t sec      = ms / 1000;
            uint32_t centisec = (ms % 1000) / 10;

            printAxisStatus(axis);

            // Serial.print("Loop: ");
            // Serial.println(avgLoopTime);

            previousPassedTime1 = passedTime;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_500HZ));
    }
}