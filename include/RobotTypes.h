#pragma once
// RobotTypes.h
// Hardware-free shared types used across libs (Navigator, MazeMapper, RobotFSM, etc.)
// No Arduino.h include here - keep this testable on native platform too.

enum class Heading {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

struct Cell {
    int x;
    int y;
};

// Add other shared types here (Direction, WallState, MissionState, etc.)
