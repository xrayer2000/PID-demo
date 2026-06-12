// Axis.cpp
#include "main.h"
#include "axis.h"

const float   readInterval  = 500;  // microseconds

AxisState axis;
PID_Controller pid( &axis.absoluteAngle, &axis.stepVelocity, &settings.setPoint);

void updateAxis(AxisState& axis, PID_Controller& pid)
{
    updateSensorValues(axis);

    //Stepper motor control
    //updateMicrostepCycle();
    updateMode(axis, pid);

    if (settings.systemMode != MODE_RPM)
    {
        if (pid.Compute())
            setAngle(axis);
    }
}

void setAngle(AxisState& axis)
{
    static bool enabled = false;

    const float ON_THRESHOLD  = 8.0f;  // start moving
    const float OFF_THRESHOLD = 1.0f;  // stop moving

    float v = fabs(axis.stepVelocity);

    if (!enabled && v > ON_THRESHOLD) {
        enabled = true;
        digitalWrite(EN_PIN, LOW); // enable driver
    } else if (enabled && v < OFF_THRESHOLD) {
        enabled = false;
        setStepFrequency(0);
        axis.stepVelocity = 0;
        digitalWrite(EN_PIN, HIGH); // disable driver
    }

    if (enabled) {
        setDirection(axis.stepVelocity);
        setStepFrequency(v);
    }
}

void updateSensorValues(AxisState& axis)
{
    uint32_t now = micros();
    uint32_t dt  = now - lastReadTime;

    if (dt < readInterval) return;
    lastReadTime = now;

    static uint16_t prevRaw     = 0;
    static bool     initialized = false;

    uint16_t rawAngle = readAS5600RawFast();

    if (!initialized) {
        prevRaw     = rawAngle;
        initialized = true;
        return;
    }

    int16_t diff = rawAngle - prevRaw;
    prevRaw = rawAngle;

    if      (diff >  2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;

    axis.absoluteAngle += diff * 0.087890625f;
    axis.angle          = axis.absoluteAngle;

    float dt_sec = dt * 1e-6f;

    if (dt_sec > 0.0f) {
        float rpm_instant = (diff / 4096.0f) / dt_sec * 60.0f;
        axis.rpm = Filter(rpm_instant, axis.rpm, 0.99f, 100.0f);
    }
}

void setStepFrequency(uint32_t steps_per_sec)
{
    axis.stepFrequency = steps_per_sec;
    if (steps_per_sec == 0) {
        stepTimer->pause();
        digitalWrite(STEP_PIN_1, LOW);
        return;
    }
    stepTimer->setOverflow(steps_per_sec, HERTZ_FORMAT);
    stepTimer->refresh();  // force immediate reload //// !!!!!VIKTIG
    stepTimer->resume();
}

void setDirection(int velocity)
{
    digitalWrite(DIR_PIN_1, velocity >= 0);
}

void updateMode(AxisState& axis, PID_Controller& pid)
{
    static int lastMode = -1;

    if (settings.systemMode != lastMode) {
        axis.absoluteAngle = 0;      // reset once when mode changes
        lastMode      = settings.systemMode;
    }

    switch (settings.systemMode) {
        case MODE_OSCILATION: {
            uint32_t period_ms  = settings.period * 1000.0f;
            uint32_t phase      = millis() % period_ms;
            float    newSetPoint = (phase < period_ms / 2.0f)
                                   ? settings.amplitude
                                   : -settings.amplitude;

            // Reset integral whenever setpoint flips
            if ((newSetPoint > 0) != (settings.setPoint > 0))
                pid.ResetIntegral();

            settings.setPoint = newSetPoint;
        } break;

        case MODE_SETPOINT:
            // nothing to compute
        break;

        case MODE_RPM: {
            float steps_per_sec = abs(settings.setPoint) / 60.0f * steps_per_rev;

            static float lastSteps = -1;
            if (steps_per_sec != lastSteps) {
                setDirection(settings.setPoint >= 0 ? 1 : -1);
                setStepFrequency((uint32_t)steps_per_sec);
                lastSteps = steps_per_sec;
            }
            axis.stepVelocity = settings.setPoint;
        } break;
    }
}

void printAxisStatus(const AxisState& axis)
{
    uint32_t ms       = millis();
    uint32_t sec      = ms / 1000;
    uint32_t centisec = (ms % 1000) / 10;

     Serial.printf(
                "Time:%4lu.%02lu\t | SetPoint:%6ld | Angle:%6.2f | RPM:%6ld | Vel:%6ld | Kp:%4ld | Ki:%4ld | Kd:%4ld | Mode:%s | Micro:1/%lu | Loop:%4lu | LoopFreq:%6lu\n",
                sec, centisec,
                (int32_t)settings.setPoint,
                axis.absoluteAngle,
                (int32_t)axis.rpm,
                (int32_t)axis.stepVelocity,
                (int32_t)settings.Kp,
                (int32_t)settings.Ki,
                (int32_t)settings.Kd,
                systemModeToString(settings.systemMode),
                (uint32_t)microstepping,
                avgLoopTime,
                1000000UL / avgLoopTime
            );
}