#pragma once
// ============================================================================
// MotorDriver
//
// Thin, reusable wrapper around a single H-bridge channel (IN1, IN2, PWM).
// One instance per motor -- the robot has two, wired up in main.cpp.
// Keeping this class dumb (no notion of "left/right" or "forward cell")
// keeps it reusable on any future robot with the same driver module.
// ============================================================================

#include <Arduino.h>

class MotorDriver {
public:
    MotorDriver(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin);

    void begin();

    // speed range: -255 (full reverse) .. +255 (full forward), 0 = stop.
    void setSpeed(int16_t speed);
    void stop();

private:
    uint8_t _in1, _in2, _pwm;
};
