#pragma once
// ============================================================================
// MazeUltrasonicSensor
//
// Single trig/echo ultrasonic ranging sensor (e.g. HC-SR04). One instance
// per sensor -- the robot has three (front, left, right).
// ============================================================================

#include <Arduino.h>

class MazeUltrasonicSensor {
public:
    MazeUltrasonicSensor(uint8_t trigPin, uint8_t echoPin, float maxRangeCm);

    void begin();

    // Returns distance in cm, or maxRangeCm if no echo was detected
    // (open space / out of range) so callers can treat "no wall" uniformly.
    float readDistanceCm() const;

private:
    uint8_t _trig, _echo;
    float _maxRangeCm;
};
