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

    float getInput() const;
    float getSetPoint() const;
    float getError() const;
    float getP() const;
    float getI() const;
    float getD() const;
    float getOutput() const;
    float getKp() const;
    float getKi() const;
    float getKd() const;

private:
    float* T;
    float* U;
    float* setPoint;

    float kp;
    float ki;
    float kd;

    float P = 0.0f;
    float I = 0.0f;
    float D = 0.0f;
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