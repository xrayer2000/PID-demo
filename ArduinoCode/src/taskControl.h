#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>

extern TaskHandle_t taskControlHandle;

void taskControl(void *pvParameters);