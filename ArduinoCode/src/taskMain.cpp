#include "main.h"
#include "taskMain.h"

void taskMain(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (true)
    {
        float passedTime = millis() * 0.001f;

        updateSettings();

        uint32_t t = btnOk.consumeTouchMs();
        if (t != 0) {
            timeLastTouched = passedTime / 60.0f;
        }

        //loop time measurement
        //----------------------------------------------------------------------------------
        now = micros();

        loopCount++;

        if (loopCount == 1)
            last = now;

        if (loopCount >= 100) {
            sumLoopTime = now - last;
            avgLoopTime = sumLoopTime / (loopCount - 1);
            loopCount   = 0;
            last        = now; // ← you reset count but never reset `last`!
        }
        //----------------------------------------------------------------------------------

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_100HZ));
    }
}