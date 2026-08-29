#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float outputMin, float outputMax)
    : _kp(kp), _ki(ki), _kd(kd), _outMin(outputMin), _outMax(outputMax),
      _integral(0.0f), _prevError(0.0f), _firstCall(true) {}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
    _firstCall = true;
}

float PIDController::compute(float error, float dtSec) {
    if (dtSec <= 0.0f) dtSec = 0.001f; // guard against div-by-zero on the first call

    _integral += error * dtSec;

    float derivative = 0.0f;
    if (!_firstCall) {
        derivative = (error - _prevError) / dtSec;
    }
    _firstCall = false;
    _prevError = error;

    float output = _kp * error + _ki * _integral + _kd * derivative;
    if (output > _outMax) output = _outMax;
    if (output < _outMin) output = _outMin;
    return output;
}
