#pragma once

#include <Arduino.h>
#include <TMCStepper.h>
 
extern HardwareSerial SerialTMC1;
extern HardwareSerial SerialTMC2;
extern TMC2209Stepper driver1;
extern TMC2209Stepper driver2;
constexpr float R_SENSE = 0.10f;

void initTMC2209();
float csToMilliamps(uint8_t cs, float rSense, bool vsenseHigh);