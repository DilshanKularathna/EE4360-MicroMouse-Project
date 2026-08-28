#include <Arduino.h>
#include "config.h"
#include "RobotTypes.h"
#include "RobotFSM.h"

// main.cpp - composition root ONLY.
// Wire up MotorDriver, Encoder, UltrasonicSensor, LineSensorArray,
// GyroMPU6050, PIDController, MazeMapper, Navigator, BridgeHandler
// and pass them into RobotFSM here. No algorithm logic should live
// in this file.

RobotFSM robot;

void setup() {
    // robot.begin();
}

void loop() {
    // robot.update();
}
