// Axis.h
#include <AS5600.h>
#include "../TrapProfile/TrapProfile.h"   
#pragma once

//--------------------------------------------------------------------------------------------------
// Axis mode
//--------------------------------------------------------------------------------------------------
enum AxisMode {
    MODE_posControl,
    MODE_posControl_OSC,
    MODE_velControl,
    MODE_velControl_OSC
};

enum AxisId {
    AXIS_1,
    AXIS_2
};

//--------------------------------------------------------------------------------------------------
// Types
//--------------------------------------------------------------------------------------------------

struct AxisPins
{
    uint8_t dirPin;
    uint8_t stepPin;
    uint8_t enPin;
};

struct AxisSettings
{
    // Target
    float targetSetPoint = 0.0f; //users desired target

    // Controller
    float Kp = 5.0f;
    float Ki = 1.0f;
    float Kd = 0.0f;

    // Position profile
    float posProMaxVel   = 400.0f; // degrees per second
    float posProMaxAccel = 600.0f; // degrees per second²

    // Velocity profile
    float velProMaxVel   = 100.0f; // degrees per second
    float velProMaxAccel = 50.0f; // degrees per second²

    // Oscillation
    float amplitude = 90.0f;
    float period    = 3.5f;

    // Mode
    AxisMode axisMode = MODE_posControl;

    // Motor
    uint16_t stepsPerRev = 200;
    uint16_t microsteps  = 256;

    uint16_t totalStepsPerRev() const {
        return stepsPerRev * microsteps;
    }

    // Hardware
    AxisPins pins;
};

struct AxisState
{
    // Position / velocity state
    float controlSetPoint   = 0.0f; // the setpoint that the controller is trying to achieve
    float pos               = 0.0f; // axis position in degrees
    float absPos            = 0.0f; // axis absolute position in degrees (from sensor)
    float vel               = 0.0f; // axis velocity in degrees per second
    float prevVel           = 0.0f;
    float acc               = 0.0f; // axis acceleration in degrees per second²
    float commandedVel      = 0.0f; // the velocity that the controller is commanding to the motor
    bool  moving            = false;

    // Sensor
    AS5600* measuredPos = nullptr;
    uint32_t lastReadTime = 0;
    uint16_t prevRaw = 0;
    bool initialized = false;
    uint32_t velWindowStart = 0;   // micros() timestamp when current velocity window began
    float    velAccumDeg    = 0.0f; // accumulated deg_delta within the current velocity window

    // Hardware
    HardwareTimer* stepTimer = nullptr;
    uint32_t       stepChannel = 1;   // NYTT: TIM_CHANNEL_1, TIM_CHANNEL_2 etc.
    volatile uint32_t stepFrequency = 0;
    bool enabled = false;

    // Configuration
    AxisSettings settings;
    AxisId axisId;

    // Motion profiles
    TrapProfile posProfile;
    TrapProfile velProfile;
    float profileTarget = 0.0f;

    // Runtime control
    int lastMode = -1;
    float lastSteps = -1.0f;

    // Last-known settings (for change detection)
    float lastSetPoint;
    float lastKp;
    float lastKi;
    float lastKd;
    float lastAmplitude;
    float lastPeriod;
};

//--------------------------------------------------------------------------------------------------
// Global objects
//--------------------------------------------------------------------------------------------------

extern AxisState axis1;
extern AxisState axis2;
extern AS5600 sensor1;
extern AS5600 sensor2;
extern PID_Controller posPid1;
extern PID_Controller velPid1;
extern PID_Controller posPid2;
extern PID_Controller velPid2;

extern AxisState* selectedAxis;

extern const float readInterval;
// Mode
const char* axisModeToString(AxisMode mode);

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------
void initPID(PID_Controller& pid, AxisState& axis);
void initAxisSettings(AxisState& axis, AxisMode mode, float setPoint, float kp, float ki, float kd, float amplitude, float period);
void updatePIDTunings(const AxisState& axis, PID_Controller& pid);

void updateAxis(AxisState& axis, PID_Controller& posPid, PID_Controller& velPid);
void updateSensorValues(AxisState& axis);
void updateMode(AxisState& axis);

void initSensor(AxisState& axis);
void updateMotor(AxisState& axis);
void setDirection(AxisState& axis, int velocity);
void setStepFrequency(AxisState& axis, float freq);
void initStepTimer(AxisState& axis, TIM_TypeDef* timer, uint32_t channel, uint32_t stepPin);

void microstepTest(AxisState& axis);
void printAxisStatus(const AxisState& axis);
void printAllAxisStatus();
void plotAxisStatus(const AxisState& axis);

const char* axisModeToString(AxisMode mode);
const char* axisIdToString(AxisId axisId);