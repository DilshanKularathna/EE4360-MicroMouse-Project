#pragma once
// ============================================================================
// PIDController
//
// Small, generic PID controller reused for both heading-hold (straight
// driving / turns, driven by the gyro) and line-following (driven by the
// IR array). Keeping this generic and hardware-agnostic means the same
// tested class backs two very different control loops.
// ============================================================================

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outputMin, float outputMax);

    void setGains(float kp, float ki, float kd);
    void reset();

    // error = setpoint - measurement, computed by the caller.
    // dtSec = elapsed time in seconds since the previous call.
    float compute(float error, float dtSec);

private:
    float _kp, _ki, _kd;
    float _outMin, _outMax;
    float _integral;
    float _prevError;
    bool _firstCall;
};
