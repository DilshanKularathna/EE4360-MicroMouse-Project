#pragma once
// ============================================================================
// MazeBridgeHandler
//
// Handles the one behavior that's unlike normal grid-maze driving: PID
// line-following along the 3cm black line across the bridge-out tile, the
// external bridge span, the bridge-in tile, and the post-entry corridor
// (which "may include bends/turns" per the brief before the floor reverts
// to the standard black maze floor).
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "MazeMotorDriver.h"
#include "MazeLineSensorArray.h"
#include "MazePIDController.h"

class MazeBridgeHandler {
public:
    MazeBridgeHandler(MazeMotorDriver &leftMotor, MazeMotorDriver &rightMotor, MazeLineSensorArray &lineSensors);

    // Blocks until the robot has fully crossed the bridge and its
    // corridor and the sensor array reports the standard black maze floor
    // again (i.e. it has arrived at a normal cell of Section B). Returns
    // false if MAX_BRIDGE_STEPS_MS elapses first, as a safety net against
    // getting stuck on a lost line.
    bool crossBridge();

private:
    MazeMotorDriver &_leftMotor;
    MazeMotorDriver &_rightMotor;
    MazeLineSensorArray &_lineSensors;
    MazePIDController _linePid;

    static const unsigned long MAX_BRIDGE_TIME_MS = 15000; // TODO: tune vs. actual bridge+corridor length
    static const uint8_t BRIDGE_BASE_SPEED = 110;           // TODO: tune -- slower than open-maze driving for the incline
};
