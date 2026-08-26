#pragma once
#include <Arduino.h>
#include <math.h>

struct TrapProfile
{
    enum class Type {
        POSITION,
        VELOCITY
    };

    Type type = Type::POSITION;

    float startValue = 0.0f;
    float target = 0.0f;

    // POSITION: maxVel   = velocity limit   (units/s)
    //           maxAccel = acceleration limit (units/s^2)
    // VELOCITY: maxVel   = acceleration limit (units/s^2)  <- intermediate cap, one derivative up
    //           maxAccel = jerk limit         (units/s^3)
    // Same trapezoidal-ramp shape applies to both; only the physical meaning
    // of the ramped quantity (and its two limits) shifts by one derivative,
    // so a single evaluator below serves both types.
    float maxVel = 0.0f;
    float maxAccel = 0.0f;

    float t_accel = 0.0f;
    float t_flat = 0.0f;
    float totalTime = 0.0f;

    uint32_t startTime_ms = 0;
    bool active = false;

    void start(float from, float to, float vMax, float aMax, uint32_t now_ms)
    {
        startValue = from;
        target = to;
        maxVel = fabsf(vMax);
        maxAccel = fabsf(aMax);
        startTime_ms = now_ms;
        active = true;

        float distance = fabsf(to - from);

        if (distance <= 0.0f || maxAccel <= 0.0f) {
            active = false;
            t_accel = 0.0f;
            t_flat = 0.0f;
            totalTime = 0.0f;
            return;
        }

        float t_acc = maxVel / maxAccel;
        float d_acc = 0.5f * maxAccel * t_acc * t_acc;

        if (2.0f * d_acc >= distance) {
            t_acc = sqrtf(distance / maxAccel);
            t_flat = 0.0f;
        } else {
            t_flat = (distance - 2.0f * d_acc) / maxVel;
        }

        t_accel = t_acc;
        totalTime = 2.0f * t_accel + t_flat;
    }

    float getPosition(uint32_t now_ms) { return evaluate(now_ms); }
    float getVelocity(uint32_t now_ms) { return evaluate(now_ms); }

private:
    // Shared trapezoidal-ramp evaluator. For POSITION this ramps position
    // with an accel-limited (trapezoidal-velocity) profile. For VELOCITY it
    // ramps velocity the same way, which — one derivative up — makes it a
    // jerk-limited (trapezoidal-acceleration / S-curve) velocity profile.
    float evaluate(uint32_t now_ms)
    {
        if (!active) return target;

        float t = (now_ms - startTime_ms) * 0.001f;
        float dir = (target >= startValue) ? 1.0f : -1.0f;

        if (t >= totalTime) {
            active = false;
            return target;
        }

        float vPeak = maxAccel * t_accel;
        float distance;

        if (t < t_accel) {
            distance = 0.5f * maxAccel * t * t;
        }
        else if (t < t_accel + t_flat) {
            distance = 0.5f * maxAccel * t_accel * t_accel
                     + vPeak * (t - t_accel);
        }
        else {
            float t_dec = t - t_accel - t_flat;

            distance = 0.5f * maxAccel * t_accel * t_accel
                     + vPeak * t_flat
                     + vPeak * t_dec
                     - 0.5f * maxAccel * t_dec * t_dec;
        }

        return startValue + dir * distance;
    }
};