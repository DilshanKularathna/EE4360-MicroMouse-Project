#pragma once
// ============================================================================
// MazeNavigator
//
// Everything that requires moving the physical robot: turning to a
// heading, driving one cell forward, sensing walls with the ultrasonics,
// and the two maze-solving behaviors built on top of a MazeMapper:
//
//   exploreAndFindGoal()  - DFS-with-backtrack exploration of an unknown
//                            maze, stopping as soon as a caller-supplied
//                            "goal detected" test succeeds (task 1 & 2 in
//                            the brief: find the bridge exit / find the
//                            finish tile).
//   runShortestPath()     - once the goal cell is known and (at least
//                            partially) mapped, flood-fill from the goal
//                            and drive the shortest known route to it
//                            (the "path optimization" bonus).
//
// MazeNavigator does NOT own a MazeMapper -- it receives one by reference for
// each call, so the exact same movement/exploration code is reused for
// Section A (4x4) and Section B (9x9) by simply passing a different map.
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "RobotTypes.h"
#include "MazeMapper.h"
#include "MazeMotorDriver.h"
#include "MazeEncoder.h"
#include "MazeUltrasonicSensor.h"
#include "MazeLineSensorArray.h"
#include "MazeGyroMPU6050.h"
#include "MazePIDController.h"

// Plain function pointer rather than std::function: AVR-GCC's bundled
// libstdc++ does not reliably provide <functional>, and a raw pointer
// avoids the heap allocation std::function can incur for captures -- both
// good reasons to avoid it on a memory-constrained microcontroller anyway.
// Pass the MazeLineSensorArray explicitly instead of capturing it.
typedef bool (*GoalDetectorFn)(MazeLineSensorArray &lineSensors);

class MazeNavigator {
public:
    MazeNavigator(MazeMotorDriver &leftMotor, MazeMotorDriver &rightMotor,
              MazeEncoder &leftEncoder, MazeEncoder &rightEncoder,
              MazeUltrasonicSensor &frontUs, MazeUltrasonicSensor &leftUs, MazeUltrasonicSensor &rightUs,
              MazeGyroMPU6050 &gyro);

    void begin();
    void stopMotors();

    // --- Low-level movement primitives ---------------------------------
    // Rotates in place to face `target`, using the gyro for closed-loop
    // heading control. Updates the internally tracked heading.
    void turnToHeading(Heading target);

    // Drives forward exactly one cell (CELL_SIZE_MM), holding heading with
    // the gyro and using encoder ticks to know when to stop.
    void driveOneCellForward();

    // Reads all three ultrasonics from the current pose and records any
    // detected walls into `maze` at (x, y), relative to `heading`.
    void senseWalls(MazeMapper &maze, uint8_t x, uint8_t y, Heading heading);

    // --- High-level behaviors --------------------------------------------
    // Explores `maze` outward from (startX, startY) with a depth-first
    // search that backtracks at dead ends, stopping as soon as
    // isGoalDetected() returns true. On success, returns true and writes
    // the final (goal) cell/heading into the out-parameters.
    bool exploreAndFindGoal(MazeMapper &maze, uint8_t startX, uint8_t startY, Heading startHeading,
                             MazeLineSensorArray &lineSensors, GoalDetectorFn isGoalDetected,
                             uint8_t &outGoalX, uint8_t &outGoalY, Heading &outFinalHeading);

    // Runs flood-fill from (goalX, goalY) over the walls learned so far in
    // `maze`, then physically drives the shortest known route there,
    // starting from (startX, startY) / startHeading.
    void runShortestPath(MazeMapper &maze, uint8_t startX, uint8_t startY, Heading startHeading,
                          uint8_t goalX, uint8_t goalY);

private:
    MazeMotorDriver &_leftMotor;
    MazeMotorDriver &_rightMotor;
    MazeEncoder &_leftEncoder;
    MazeEncoder &_rightEncoder;
    MazeUltrasonicSensor &_frontUs;
    MazeUltrasonicSensor &_leftUs;
    MazeUltrasonicSensor &_rightUs;
    MazeGyroMPU6050 &_gyro;

    MazePIDController _headingPid;

    void driveWithHeadingHold(float targetHeadingDeg, int16_t baseSpeed, long targetEncoderTicks);
};
