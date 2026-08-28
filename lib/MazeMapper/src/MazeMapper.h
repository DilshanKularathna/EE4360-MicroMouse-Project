#pragma once
// MazeMapper
// Pure flood-fill maze mapping / shortest-path logic.
// Deliberately hardware-free (no Arduino.h) so it can be unit tested
// under the "native" PlatformIO environment (see test/test_maze_mapper).

#include "RobotTypes.h"

class MazeMapper {
public:
    // TODO: setWall(), floodFill(), getShortestPath(), reset()
};
