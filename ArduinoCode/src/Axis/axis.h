// Axis.h
#pragma once

//--------------------------------------------------------------------------------------------------
// Types
//--------------------------------------------------------------------------------------------------

struct AxisState
{
    float angle = 0.0f;
    float absoluteAngle = 0.0f;
    float rpm = 0.0f;
    float stepVelocity = 0.0f;

    volatile uint32_t stepFrequency = 0;
};

//--------------------------------------------------------------------------------------------------
// Global objects
//--------------------------------------------------------------------------------------------------

extern AxisState axis;
extern PID_Controller pid;
extern const float readInterval;

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

void updateAxis(AxisState& axis, PID_Controller& pid);
void updateSensorValues(AxisState& axis);
void updateMode(AxisState& axis, PID_Controller& pid);

void setAngle(AxisState& axis);
void setDirection(int velocity);
void setStepFrequency(uint32_t freq);

void printAxisStatus(const AxisState& axis);