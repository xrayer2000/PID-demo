#include "PID_Controller.h"
#include <math.h>

PID_Controller::PID_Controller(float* input, float* output, float* setpoint)
{
    T = input;
    U = output;
    AngleSet = setpoint;

    kp = 30.0;
    ki = 0.0;
    kd = 0.0;

    I = 0.0;
    Imax = 1000.0;

    outMin = -4095.0;
    outMax = 4095.0;

    Ts_us = 1000;     // 1 ms loop
    lastTime = 0;

    lastInput = 0.0;
    Df = 0.0;         // filtered derivative
    dFilterAlpha = 0.9;   // derivative filter strength
}

void PID_Controller::SetTunings(float Kp, float Ki, float Kd)
{
    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void PID_Controller::ResetIntegral()
{
    I = 0.0;
}

void PID_Controller::SetOutputLimits(float minOut, float maxOut)
{
    if (minOut >= maxOut) return;

    outMin = minOut;
    outMax = maxOut;

    if (*U > outMax) *U = outMax;
    if (*U < outMin) *U = outMin;
}

void PID_Controller::SetIntegralLimit(float maxI)
{
    if (maxI < 0) return;
    Imax = maxI;
}

void PID_Controller::SetSampleTime(float ms)
{
    if (ms <= 0) return;
    Ts_us = (uint32_t)(ms * 1000.0);
}

bool PID_Controller::Compute()
{
    uint32_t now = micros();

    // initialization
    if (lastTime == 0)
    {
        lastTime = now;
        lastInput = *T;
        return false;
    }

    uint32_t dt = now - lastTime;
    if (dt < Ts_us) return false;

    lastTime = now;

    float Ts = dt * 1e-6;
    if (Ts <= 0.0) return false;

    float input = *T;
    if (!isfinite(input)) return false;

    float error = *AngleSet - input;

    // --- proportional
    float P = kp * error;

    // --- derivative on measurement
    float dInput = (input - lastInput) / Ts;
    float D_raw = -kd * dInput;

    // --- low-pass filtered derivative
    Df = dFilterAlpha * Df + (1.0 - dFilterAlpha) * D_raw;
    float D = Df;

    // --- integral candidate
    float I_candidate = I + ki * error * Ts;

    if (I_candidate > Imax) I_candidate = Imax;
    if (I_candidate < -Imax) I_candidate = -Imax;

    // --- tentative output
    float output = P + I_candidate + D;

    // --- anti-windup (conditional integration)
    if (output > outMax)
    {
        output = outMax;
        if (error < 0) I = I_candidate;
    }
    else if (output < outMin)
    {
        output = outMin;
        if (error > 0) I = I_candidate;
    }
    else
    {
        I = I_candidate;
    }

    *U = output;

    lastInput = input;

    return true;
}