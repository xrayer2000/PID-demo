#pragma once
#include <Arduino.h>

class PID_Controller {
public:
    PID_Controller(float* input,
                   float* output,
                   float* setpoint);

    void SetTunings(float Kp, float Ki, float Kd);

    void SetOutputLimits(float minOut, float maxOut);
    void SetIntegralLimit(float maxI);
    void ResetIntegral();
    void SetSampleTime(float ms);

    void SetStepsPerRev(uint16_t steps) { stepsPerRev = steps; }
    bool Compute();

private:
    float* T;
    float* U;
    float* AngleSet;

    float kp;
    float ki;
    float kd;

    float I;
    float Imax;

    float lastInput;

    float Df;
    float dFilterAlpha;

    float outMin;
    float outMax;

    float deadband = 0.0f;
    uint16_t stepsPerRev = 200;

    uint32_t Ts_us;
    uint32_t lastTime;
};