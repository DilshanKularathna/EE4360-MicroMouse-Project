#include "MazeMotorDriver.h"

MazeMotorDriver::MazeMotorDriver(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin)
    : _in1(in1Pin), _in2(in2Pin), _pwm(pwmPin) {}

void MazeMotorDriver::begin() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_pwm, OUTPUT);
    stop();
}

void MazeMotorDriver::setSpeed(int16_t speed) {
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    } else if (speed < 0) {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    } else {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
    }

    analogWrite(_pwm, abs(speed));
}

void MazeMotorDriver::stop() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    analogWrite(_pwm, 0);
}
