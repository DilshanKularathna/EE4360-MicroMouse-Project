#pragma once
// ============================================================================
// RobotFSM
//
// Top-level orchestrator. Owns the two MazeMapper instances (Section A 4x4,
// Section B 9x9) and drives the mission phases described in the brief:
//
//   1. Explore Section A, detecting the bridge-approach marker as the goal.
//   2. Re-run Section A via the shortest known path (path optimization).
//   3. Cross the bridge + corridor via line-following.
//   4. Explore Section B, detecting the white FINISH tile as the goal.
//   5. Re-run Section B via the shortest known path to the finish.
//
// This class contains the "what happens when" logic; all physical
// movement/sensing logic lives in Navigator/BridgeHandler/LineSensorArray,
// which RobotFSM simply calls in sequence. That separation is what makes
// each phase easy to test, re-run, or reorder independently.
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "RobotTypes.h"
#include "MazeMapper.h"
#include "Navigator.h"
#include "BridgeHandler.h"
#include "LineSensorArray.h"

class RobotFSM {
public:
    RobotFSM(Navigator &navigator, BridgeHandler &bridge, LineSensorArray &lineSensors);

    // Blocks the button press (or an immediate start, if you remove that
    // wait) then runs every phase of the mission to completion. Kept as a
    // single blocking call for readability -- each phase already blocks
    // internally (see Navigator/BridgeHandler), so there is no benefit to
    // a non-blocking loop() style state machine here.
    void run();

    RobotState getState() const { return _state; }

private:
    Navigator &_navigator;
    BridgeHandler &_bridge;
    LineSensorArray &_lineSensors;

    MazeMapper _mazeA;
    MazeMapper _mazeB;

    RobotState _state;

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
};
