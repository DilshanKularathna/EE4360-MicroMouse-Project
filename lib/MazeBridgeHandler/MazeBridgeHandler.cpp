#include "MazeBridgeHandler.h"

MazeBridgeHandler::MazeBridgeHandler(MazeMotorDriver &leftMotor, MazeMotorDriver &rightMotor, MazeLineSensorArray &lineSensors)
    : _leftMotor(leftMotor), _rightMotor(rightMotor), _lineSensors(lineSensors),
      _linePid(LINE_KP, LINE_KI, LINE_KD, -BRIDGE_BASE_SPEED, BRIDGE_BASE_SPEED) {}

bool MazeBridgeHandler::crossBridge() {
    _linePid.reset();
    unsigned long startMs = millis();
    unsigned long lastMicros = micros();

    // The line-background is white on the bridge; once we're back on the
    // standard black maze floor, allBlack()/an absence of the line pattern
    // signals arrival in Section B. Require a short confirmation window so
    // a momentary floor seam or gap in the line doesn't trigger a false
    // "arrived" exit.
    const uint8_t CONFIRM_SAMPLES = 5;
    uint8_t offLineStreak = 0;

    while (millis() - startMs < MAX_BRIDGE_TIME_MS) {
        _lineSensors.readAll();

        if (_lineSensors.allBlack()) {
            // Standard maze floor is plain black with no reflective line
            // -- once every sensor reads "black" uniformly for a few
            // consecutive samples, we've left the white bridge/corridor
            // surface and re-entered Section A's/B's normal floor.
            offLineStreak++;
            if (offLineStreak >= CONFIRM_SAMPLES) {
                _leftMotor.stop();
                _rightMotor.stop();
                return true;
            }
        } else {
            offLineStreak = 0;
        }

        int16_t error = _lineSensors.getLineError(); // -100..100, 0 = centered
        float dt = (micros() - lastMicros) / 1000000.0f;
        lastMicros = micros();
        float correction = _linePid.compute(static_cast<float>(error), dt);

        _leftMotor.setSpeed(static_cast<int16_t>(BRIDGE_BASE_SPEED - correction));
        _rightMotor.setSpeed(static_cast<int16_t>(BRIDGE_BASE_SPEED + correction));
    }

    // Timed out -- stop the robot rather than driving blind indefinitely.
    _leftMotor.stop();
    _rightMotor.stop();
    return false;
}
