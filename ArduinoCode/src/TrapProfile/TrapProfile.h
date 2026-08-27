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

        if (type == Type::POSITION) {

            // Position profile:
            // maxVel = velocity limit
            // maxAccel = acceleration limit

            float t_acc = maxVel / maxAccel;
            float d_acc = 0.5f * maxAccel * t_acc * t_acc;

            if (2.0f * d_acc >= distance) {

                // Triangular velocity profile
                t_acc = sqrtf(distance / maxAccel);
                t_flat = 0.0f;

            } else {

                // Trapezoidal velocity profile
                t_flat = (distance - 2.0f * d_acc) / maxVel;
            }

            t_accel = t_acc;
            totalTime = 2.0f * t_accel + t_flat;

        } else {

            // Velocity profile:
            // ramp velocity from startValue to target
            // using maxAccel as acceleration limit.

            t_accel = distance / maxAccel;
            t_flat = 0.0f;
            totalTime = t_accel;
        }
    }

    float getPosition(uint32_t now_ms)
    {
        if (type != Type::POSITION)
            return target;

        return evaluatePosition(now_ms);
    }

    float getVelocity(uint32_t now_ms)
    {
        if (type != Type::VELOCITY)
            return 0.0f;

        return evaluateVelocity(now_ms);
    }

private:

    float evaluatePosition(uint32_t now_ms)
    {
        if (!active)
            return target;

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

        } else if (t < t_accel + t_flat) {

            distance = 0.5f * maxAccel * t_accel * t_accel
                     + vPeak * (t - t_accel);

        } else {

            float t_dec = t - t_accel - t_flat;

            distance = 0.5f * maxAccel * t_accel * t_accel
                     + vPeak * t_flat
                     + vPeak * t_dec
                     - 0.5f * maxAccel * t_dec * t_dec;
        }

        return startValue + dir * distance;
    }

    float evaluateVelocity(uint32_t now_ms)
    {
        if (!active)
            return target;

        float t = (now_ms - startTime_ms) * 0.001f;

        float dir = (target >= startValue) ? 1.0f : -1.0f;

        if (t >= totalTime) {
            active = false;
            return target;
        }

        // Linear velocity ramp:
        //
        // startValue --------> target
        //
        // with constant acceleration.

        float velocity = startValue + dir * maxAccel * t;

        // Prevent overshoot
        if (dir > 0.0f && velocity > target)
            velocity = target;

        if (dir < 0.0f && velocity < target)
            velocity = target;

        return velocity;
    }
};