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

        updateAxis(axis1, posPid1, velPid1);
        updateAxis(axis2, posPid2, velPid2);

        if (shouldUpdate1) {
            // Serial.print(">plotDt:");
            // Serial.println(passedTime - previousPassedTime1);


            printAllAxisStatus();
            // plotAxisStatus(axis1);
            // plotAxisStatus(axis2);

            // Serial.print("Loop: ");
            // Serial.println(avgLoopTime);

            previousPassedTime1 = passedTime;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_200HZ));
    }
}