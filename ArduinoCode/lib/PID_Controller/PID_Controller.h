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

    uint32_t Ts_us;
    uint32_t lastTime;
};