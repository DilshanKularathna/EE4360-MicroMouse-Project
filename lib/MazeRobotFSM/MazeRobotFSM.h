#pragma once
// ============================================================================
// MazeRobotFSM
//
// Top-level orchestrator. Owns the two MazeMapper instances (Section A 4x4,
// Section B 9x9) and drives the mission phases described in the brief.
//
// With a blank EEPROM it fully explores each section, backtracks to its
// section start, drives its learned shortest path, and saves the completed
// map pair at FINISH.  After a reset before a later trial, it restores that
// map pair and immediately executes the known START-to-FINISH route.  No
// source upload or external maze information is needed between attempts.
//
// Exploration sequence:
//
//   1. Explore Section A, detecting the bridge-approach marker as the goal.
//   2. Re-run Section A via the shortest known path (path optimization).
//   3. Cross the bridge + corridor via line-following.
//   4. Explore Section B, detecting the white FINISH tile as the goal.
//   5. Re-run Section B via the shortest known path to the finish.
//
// This class contains the "what happens when" logic; all physical
// movement/sensing logic lives in MazeNavigator/MazeBridgeHandler/MazeLineSensorArray,
// which MazeRobotFSM simply calls in sequence. That separation is what makes
// each phase easy to test, re-run, or reorder independently.
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "RobotTypes.h"
#include "MazeMapper.h"
#include "MazeNavigator.h"
#include "MazeBridgeHandler.h"
#include "MazeLineSensorArray.h"
#include "MazePersistentStore.h"

class MazeRobotFSM {
public:
    MazeRobotFSM(MazeNavigator &navigator, MazeBridgeHandler &bridge, MazeLineSensorArray &lineSensors);

    // Blocks the button press (or an immediate start, if you remove that
    // wait) then runs every phase of the mission to completion. Kept as a
    // single blocking call for readability -- each phase already blocks
    // internally (see MazeNavigator/MazeBridgeHandler), so there is no benefit to
    // a non-blocking loop() style state machine here.
    void run();

    RobotState getState() const { return _state; }

private:
    MazeNavigator &_navigator;
    MazeBridgeHandler &_bridge;
    MazeLineSensorArray &_lineSensors;

    MazeMapper _mazeA;
    MazeMapper _mazeB;
    MazePersistentStore _store;

    RobotState _state;
    RunMode _mode;
    uint8_t _goalAX, _goalAY;
    uint8_t _goalBX, _goalBY;

    // TODO: set these from the officials' on-the-day announcement of the
    // bridge corner / start orientation. (0,0) with a NORTH start heading
    // is a placeholder matching "any tile, any orientation" from the brief.
    static const uint8_t START_A_X = 0;
    static const uint8_t START_A_Y = 0;
    static const Heading START_A_HEADING = Heading::NORTH;

    // Section B's entry cell is wherever the bridge-in tile lands, which is
    // announced on the day (brief: "the bridge originates from a corner
    // tile of Section A... and enters a corner tile of Section B").
    // TODO: set to match the announced bridge-in corner before competition.
    static const uint8_t START_B_X = 0;
    static const uint8_t START_B_Y = 0;
    static const Heading START_B_HEADING = Heading::NORTH;

    void setState(RobotState s) { _state = s; }
    void clearSavedMapOnLongBootPress();
    void waitForOfficialStart();
    void exploreCourse();
    void runSavedCourse();
};
