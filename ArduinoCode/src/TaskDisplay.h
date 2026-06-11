#pragma once

#include <STM32FreeRTOS.h>

// Task prototype
void TaskDisplay(void *pvParameters);

// Optional: task handle
extern TaskHandle_t taskDisplayHandle;