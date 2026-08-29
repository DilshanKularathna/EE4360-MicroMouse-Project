#include "MazeUltrasonicSensor.h"

MazeUltrasonicSensor::MazeUltrasonicSensor(uint8_t trigPin, uint8_t echoPin, float maxRangeCm)
    : _trig(trigPin), _echo(echoPin), _maxRangeCm(maxRangeCm) {}

void MazeUltrasonicSensor::begin() {
    pinMode(_trig, OUTPUT);
    pinMode(_echo, INPUT);
    digitalWrite(_trig, LOW);
}

float MazeUltrasonicSensor::readDistanceCm() const {
    digitalWrite(_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig, LOW);

    // Timeout matched to maxRangeCm so a missing echo doesn't block for the
    // full default 1s pulseIn() timeout (speed of sound ~= 0.0343 cm/us,
    // round trip -> distance*2/0.0343).
    unsigned long timeoutUs = static_cast<unsigned long>((_maxRangeCm * 2.0f / 0.0343f) + 2000);
    unsigned long durationUs = pulseIn(_echo, HIGH, timeoutUs);

    if (durationUs == 0) return _maxRangeCm; // no echo -> treat as clear/open

    float distanceCm = (durationUs * 0.0343f) / 2.0f;
    if (distanceCm > _maxRangeCm) distanceCm = _maxRangeCm;
    return distanceCm;
}
