// Axis.cpp
#include "main.h"
#include "axis.h"
#include "../TMC2209/tmc2209.h"


const float   readInterval  = 5000;  // microseconds
constexpr uint32_t VEL_WINDOW_US = 1000;   // velocity accumulation window

AS5600 sensor1(&Wire1);
AS5600 sensor2(&Wire2);

AxisState axis1 = { .measuredPos = &sensor1, .settings = { .pins = { DIR_PIN_1, STEP_PIN_1, EN_PIN } }, .axisId = AXIS_1 };
AxisState axis2 = { .measuredPos = &sensor2, .settings = { .pins = { DIR_PIN_2, STEP_PIN_2, EN_PIN } }, .axisId = AXIS_2 };

PID_Controller posPid1( &axis1.absPos, &axis1.commandedVel, &axis1.controlSetPoint);
PID_Controller velPid1(&axis1.vel, &axis1.commandedVel, &axis1.controlSetPoint);

PID_Controller posPid2( &axis2.absPos, &axis2.commandedVel, &axis2.controlSetPoint);
PID_Controller velPid2(&axis2.vel, &axis2.commandedVel, &axis2.controlSetPoint);

AxisState* selectedAxis = &axis1;

void initPID(PID_Controller& pid, AxisState& axis)
{
    constexpr float MAX_DEG_S = 3600.0f;   // was MAX_RPM = 600.0f  (600 × 6)

    pid.SetOutputLimits(-MAX_DEG_S, MAX_DEG_S);
    pid.SetIntegralLimit(1000);
    pid.SetSampleTime(5);
    pid.SetStepsPerRev(axis.settings.totalStepsPerRev());
}

void initAxisSettings(AxisState& axis, AxisMode mode, float setPoint, float kp, float ki, float kd, float amplitude, float period)
{
    axis.settings.axisMode  = mode;
    axis.settings.targetSetPoint  = setPoint;
    axis.settings.Kp        = kp;
    axis.settings.Ki        = ki;
    axis.settings.Kd        = kd;
    axis.settings.amplitude = amplitude;
    axis.settings.period    = period;
}

void initSensor(AxisState& axis)
{
    axis.measuredPos->setSlowFilter(AS5600_SLOW_FILT_16X);
    // const uint8_t AS5600_SLOW_FILT_16X      = 0;   // SF = 00 (default)
    // const uint8_t AS5600_SLOW_FILT_8X       = 1;   // SF = 01
    // const uint8_t AS5600_SLOW_FILT_4X       = 2;   // SF = 10
    // const uint8_t AS5600_SLOW_FILT_2X       = 3;   // SF = 11
    // axis.measuredPos->setHysteresis(AS5600_HYST_OFF);  // optional, avoids extra dead-banding
}

void updatePIDTunings(const AxisState& axis, PID_Controller& pid)
{
    pid.SetTunings(axis.settings.Kp, axis.settings.Ki, axis.settings.Kd);
}

// void updateAxis(AxisState& axis, PID_Controller& pid)
// {
//     updateSensorValues(axis);

//     //Stepper motor control
//     //updateMicrostepCycle();
//     updateMode(axis, pid);

//     if (axis.settings.axisMode != MODE_velControl)
//     {
//         if (pid.Compute())
//             setAngle(axis);
//     }
// }

void updateAxis(AxisState& axis, PID_Controller& posPid, PID_Controller& velPid)
{
    updateSensorValues(axis);
    updateMode(axis);

    if (axis.settings.axisMode == MODE_velControl ||
    axis.settings.axisMode == MODE_velControl_OSC) {
    velPid.Compute();
    } else {
        posPid.Compute();
    }

    updateMotor(axis);
}

void updateSensorValues(AxisState& axis)
{
    uint32_t now = micros();
    uint32_t dt  = now - axis.lastReadTime;

    if (dt < readInterval) return;
    axis.lastReadTime = now;

    uint16_t rawAngle = axis.measuredPos->rawAngle();

    if (!axis.initialized) {
        axis.prevRaw     = rawAngle;
        axis.initialized = true;
        axis.velWindowStart = now;   // NEW: init window start too
        return;
    }

    int16_t diff = rawAngle - axis.prevRaw;
    axis.prevRaw = rawAngle;

    if      (diff >  2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;

    float deg_delta = diff * 0.087890625f;   // 360° / 4096 counts
    axis.absPos += deg_delta;
    axis.pos     = axis.absPos;

    // --- velocity: accumulate over a longer window than the position sample ---
    axis.velAccumDeg += deg_delta;

    uint32_t velDt = now - axis.velWindowStart;
    if (velDt >= VEL_WINDOW_US) {                 // e.g. constexpr uint32_t VEL_WINDOW_US = 5000;
        float velDt_sec = velDt * 1e-6f;
        float degS_instant = axis.velAccumDeg / velDt_sec;
        axis.vel = Filter(degS_instant, axis.vel, 0.99f, 600.0f);

        axis.velAccumDeg     = 0.0f;
        axis.velWindowStart  = now;
    }

    // Notify display task that new sensor data is ready
    updateDisp2Flag = true;
}

void updateMode(AxisState& axis)
{

    if (axis.settings.axisMode != axis.lastMode) {
        axis.absPos = 0;      // reset once when mode changes
        axis.lastMode      = axis.settings.axisMode;
        axis.profileTarget = NAN;
    }

    switch (axis.settings.axisMode) {

        case MODE_posControl: {
            if (axis.settings.targetSetPoint != axis.profileTarget) {
                axis.posProfile.start(axis.controlSetPoint, axis.settings.targetSetPoint, axis.settings.posProMaxVel, axis.settings.posProMaxAccel, millis());
                axis.profileTarget = axis.settings.targetSetPoint;
            }

            axis.controlSetPoint = axis.posProfile.getPosition(millis());
        } break;

        case MODE_posControl_OSC: {
            uint32_t period_ms = axis.settings.period * 1000.0f;
            uint32_t phase = millis() % period_ms;
            float targetSP = (phase < period_ms / 2.0f) ? axis.settings.amplitude : -axis.settings.amplitude;

            if (targetSP != axis.profileTarget) {
                axis.posProfile.start(axis.controlSetPoint, targetSP, axis.settings.posProMaxVel, axis.settings.posProMaxAccel, millis());
                axis.profileTarget = targetSP;
            }

            axis.controlSetPoint = axis.posProfile.getPosition(millis());
        } break;

        case MODE_velControl: {

            float targetDegS = axis.settings.targetSetPoint;

            if (targetDegS != axis.profileTarget) {
                axis.velProfile.start(axis.vel, targetDegS, axis.settings.velProMaxVel, axis.settings.velProMaxAccel, millis());
                axis.profileTarget = targetDegS;
            }

            axis.controlSetPoint = axis.velProfile.getVelocity(millis());
            
            // TEMP: BLDC test — setPoint now directly represents duty %, ignore steps_per_rev
            // float duty = constrain(fabs(axis.settings.targetSetPoint), 0.0f, 100.0f);

            // setDirection(axis, axis.settings.targetSetPoint >= 0 ? 1 : -1);
            // setStepFrequency(axis, duty);
            // axis.stepVelocity = axis.settings.targetSetPoint;
            // end temp

        } break;

        case MODE_velControl_OSC: {
            uint32_t period_ms = axis.settings.period * 1000.0f;
            uint32_t phase = millis() % period_ms;

            float targetDegS = (phase < period_ms / 2.0f)
                            ? axis.settings.amplitude
                            : -axis.settings.amplitude;

            if (targetDegS != axis.profileTarget) {
                axis.velProfile.start(axis.vel, targetDegS, axis.settings.velProMaxVel, axis.settings.velProMaxAccel, millis());
                axis.profileTarget = targetDegS;
            }

            axis.controlSetPoint = axis.velProfile.getVelocity(millis());
        } break;
    }
}

void updateMotor(AxisState& axis)
{
    const float ON_THRESHOLD  = 6.0f;   // was 1.0f RPM  -> 1 RPM × 6 = 6 deg/s
    const float OFF_THRESHOLD = 3.0f;   // was 0.5f RPM  -> 0.5 RPM × 6 = 3 deg/s

    float v = fabs(axis.commandedVel);   // now deg/s

    if (!axis.enabled && v > ON_THRESHOLD) {
        axis.enabled = true;
        digitalWrite(axis.settings.pins.enPin, LOW);
    } else if (axis.enabled && v < OFF_THRESHOLD) {
        axis.enabled = false;
        setStepFrequency(axis, 0);
        axis.commandedVel = 0;
        digitalWrite(axis.settings.pins.enPin, HIGH);
    }

    if (axis.enabled) {
        setDirection(axis, axis.commandedVel);

        float steps_per_sec = v / 360.0f * axis.settings.totalStepsPerRev();   // was v/60.0f (RPM->steps/s)
        setStepFrequency(axis, steps_per_sec);
    }
}

void setStepFrequency(AxisState& axis, float steps_per_sec)
{
    axis.stepFrequency = steps_per_sec;

    if (steps_per_sec <= 0.0f)
    {
        axis.stepTimer->pause();
        digitalWrite(axis.settings.pins.stepPin, LOW);
        return;
    }

    axis.stepTimer->setOverflow(steps_per_sec, HERTZ_FORMAT);
    axis.stepTimer->refresh();
    axis.stepTimer->resume();

    //BLDC temp
    // float duty = constrain(steps_per_sec, 0.0f, 100.0f);
    // uint8_t pwmVal = (uint8_t)(duty * 2.55f);  // 0-100% -> 0-255
    // pwmVal = constrain(pwmVal, 0, 254);  // avoid the 255 edge case discussed earlier
    // analogWrite(axis.settings.pins.stepPin, pwmVal);
    // end temp
}

void setDirection(AxisState& axis, int velocity)
{
    digitalWrite(axis.settings.pins.dirPin, velocity >= 0);
}

void stepISR1()
{
    digitalWrite(axis1.settings.pins.stepPin, HIGH);
    // delayMicroseconds(3); // for DM556 to work properly, needs at least 2.5us pulse width
    digitalWrite(axis1.settings.pins.stepPin, LOW);
}

void stepISR2()
{
    digitalWrite(axis2.settings.pins.stepPin, HIGH);
    // delayMicroseconds(3); // for DM556 to work properly, needs at least 2.5us pulse width
    digitalWrite(axis2.settings.pins.stepPin, LOW);
}

void initStepTimer(AxisState& axis, TIM_TypeDef* timer, void (*isr)())
{
    pinMode(axis.settings.pins.stepPin, OUTPUT);

    axis.stepTimer = new HardwareTimer(timer);
    axis.stepTimer->setOverflow(1000, HERTZ_FORMAT);
    axis.stepTimer->attachInterrupt(isr);

    //BLDC temp
    // analogWriteFrequency(100000);  // sets PWM carrier freq for the timer(s) used by analogWrite
    // 'timer' and 'isr' params unused in this mode — call site can stay as-is
    //end temp
}

void microstepTest(AxisState& axis)
{
    const uint16_t testSteps = 200;

    updateSensorValues(axis);
    float startAngle = axis.absPos;

    for (uint16_t i = 0; i < testSteps; i++) {
        digitalWrite(axis.settings.pins.stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(axis.settings.pins.stepPin, LOW);
        delayMicroseconds(2000);

        updateSensorValues(axis);
    }

    updateSensorValues(axis);
    float delta = axis.absPos - startAngle;

    Serial.print("Microstep test: ");
    Serial.print(testSteps);
    Serial.print(" pulses -> ");
    Serial.print(delta, 2);
    Serial.println(" deg");
}

void printAxisStatus(const AxisState& axis)
{
    TMC2209Stepper& driver = (axis.axisId == AXIS_1) ? driver1 : driver2;

    const uint8_t cs      = driver.cs_actual();
    const uint8_t irun    = driver.irun();
    const uint8_t ihold   = driver.ihold();

    // VSENSE ändras normalt inte efter init
    const uint8_t vsense  = driver.vsense();

    const uint32_t tstep  = driver.TSTEP();
    const uint8_t pwm     = driver.pwm_scale_sum();

    const float current_mA = csToMilliamps(cs, R_SENSE, vsense);

    PID_Controller& pid = (axis.settings.axisMode == MODE_velControl ||
                       axis.settings.axisMode == MODE_velControl_OSC)
                    ? ((axis.axisId == AXIS_1) ? velPid1 : velPid2)
                    : ((axis.axisId == AXIS_1) ? posPid1 : posPid2);

    // Serial.printf(
    //     "Axis:%s | Mode:%s | TSP:%6.1f | CSP:%6.1f | Pos:%6.1f | Vel:%6.1f | CmdVel:%6.1f | "
    //     "Err:%6.1f | P:%6.1f | I:%6.1f | D:%6.1f | Out:%6.1f | "
    //     "Kp:%4.1f | Ki:%4.1f | Kd:%4.1f | "
    //     "Amp:%6.1f | Period:%5.1f | MaxVel:%6.1f | MaxAccel:%6.1f",
    //     axisIdToString(axis.axisId),
    //     axisModeToString(axis.settings.axisMode),
    //     axis.settings.targetSetPoint,
    //     axis.controlSetPoint,
    //     axis.absPos,
    //     axis.vel,
    //     axis.commandedVel,
    //     pid.getError(),
    //     pid.getP(),
    //     pid.getI(),
    //     pid.getD(),
    //     pid.getOutput(),
    //     pid.getKp(),
    //     pid.getKi(),
    //     pid.getKd(),
    //     axis.settings.amplitude,
    //     axis.settings.period,
    //     axis.settings.posProMaxVel,
    //     axis.settings.posProMaxAccel
    // );

    Serial.printf(
        "Axis:%s | ControlMode:%s | TSP:%6ld | CSP:%6ld | Pos:%6.2f | Vel:%6ld | CmdVel:%6ld | "
        "Kp:%4ld | Ki:%4ld | Kd:%4ld | Micro:1/%lu | "
        "CS:%2u | IRUN:%2u | IHOLD:%2u | VS:%u | "
        "TSTEP:%6lu | PWM:%3u | I:%4.0fmA ",

        axisIdToString(axis.axisId),
        axisModeToString(axis.settings.axisMode),
        (int32_t)axis.settings.targetSetPoint,
        (int32_t)axis.controlSetPoint,
        axis.absPos,
        (int32_t)axis.vel,
        (int32_t)axis.commandedVel,
        (int32_t)axis.settings.Kp,
        (int32_t)axis.settings.Ki,
        (int32_t)axis.settings.Kd,
        (uint32_t)axis.settings.microsteps,

        cs,
        irun,
        ihold,
        vsense,
        (unsigned long)tstep,
        pwm,
        current_mA

    );
}

void printAllAxisStatus()
{
    uint32_t ms       = millis();
    uint32_t sec      = ms / 1000;
    uint32_t centisec = (ms % 1000) / 10;
    
    Serial.printf("Time:%4lu.%02lu\t  | ", sec, centisec);
    printAxisStatus(axis1);
    // Serial.print(" | ");
    // printAxisStatus(axis2);
    Serial.println();
}

void plotAxisStatus(const AxisState& axis)
{
    const char* id = axisIdToString(axis.axisId);   // e.g. "axis1" / "axis2"

    Serial.print(">pos_");             Serial.print(id); Serial.print(':');
    Serial.println(axis.absPos);

    Serial.print(">targetSetPoint_");  Serial.print(id); Serial.print(':');
    Serial.println(axis.settings.targetSetPoint);

    Serial.print(">vel_");             Serial.print(id); Serial.print(':');
    Serial.println(axis.vel);

    Serial.print(">CmdVel_");          Serial.print(id); Serial.print(':');
    Serial.println(axis.commandedVel);
}