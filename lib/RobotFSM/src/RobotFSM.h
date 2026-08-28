#pragma once
// RobotFSM
// Top-level mission state machine:
//   EXPLORE_SECTION_A -> CROSS_BRIDGE -> EXPLORE_SECTION_B
//   -> RUN_SHORTEST_PATH -> FINISHED
// Delegates to Navigator, BridgeHandler, and MazeMapper as needed.

enum class MissionState {
    EXPLORE_SECTION_A,
    CROSS_BRIDGE,
    EXPLORE_SECTION_B,
    RUN_SHORTEST_PATH,
    FINISHED
};

class RobotFSM {
public:
    // TODO: begin(), update() [call once per loop()]
};
