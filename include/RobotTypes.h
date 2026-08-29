#pragma once
// ============================================================================
// RobotTypes.h
//
// Shared, hardware-independent types. Deliberately does NOT include
// <Arduino.h> so that any file which only needs these types (e.g.
// MazeMapper) can also be compiled and unit-tested on a host PC via
// `pio test -e native`.
// ============================================================================

#include <stdint.h>

// Compass heading of the robot within a maze section.
enum class Heading : uint8_t {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

// Bitmask flags for the four walls of a single cell.
enum WallDir : uint8_t {
    WALL_NORTH = 0x1,
    WALL_EAST  = 0x2,
    WALL_SOUTH = 0x4,
    WALL_WEST  = 0x8
};

struct Cell {
    int8_t x;
    int8_t y;

    bool operator==(const Cell &other) const { return x == other.x && y == other.y; }
    bool operator!=(const Cell &other) const { return !(*this == other); }
};

// Overall mission state, driven by RobotFSM.
enum class RobotState : uint8_t {
    INIT,
    WAIT_FOR_START,
    SECTION_A_EXPLORE,
    SECTION_A_SPEED_RUN,
    BRIDGE_CROSSING,
    SECTION_B_EXPLORE,
    SECTION_B_SPEED_RUN,
    FINISHED,
    ERROR
};

// ---------------------------------------------------------------------------
// Small, pure direction-math helpers shared by MazeMapper and Navigator.
// Keeping these inline+header-only avoids a pointless extra .cpp/.o for a
// handful of one-liners while still being reusable everywhere.
// ---------------------------------------------------------------------------

inline Heading turnRightOf(Heading h) {
    return static_cast<Heading>((static_cast<uint8_t>(h) + 1) % 4);
}

inline Heading turnLeftOf(Heading h) {
    return static_cast<Heading>((static_cast<uint8_t>(h) + 3) % 4);
}

inline Heading oppositeOf(Heading h) {
    return static_cast<Heading>((static_cast<uint8_t>(h) + 2) % 4);
}

inline WallDir headingToWallDir(Heading h) {
    switch (h) {
        case Heading::NORTH: return WALL_NORTH;
        case Heading::EAST:  return WALL_EAST;
        case Heading::SOUTH: return WALL_SOUTH;
        default:             return WALL_WEST;
    }
}

// Returns the neighboring cell one step away from `c` in heading `h`.
// Does NOT check maze bounds -- callers must validate with the maze size.
inline Cell neighborInDirection(Cell c, Heading h) {
    switch (h) {
        case Heading::NORTH: return Cell{c.x, static_cast<int8_t>(c.y - 1)};
        case Heading::SOUTH: return Cell{c.x, static_cast<int8_t>(c.y + 1)};
        case Heading::EAST:  return Cell{static_cast<int8_t>(c.x + 1), c.y};
        default:              return Cell{static_cast<int8_t>(c.x - 1), c.y};
    }
}

// Signed turn amount in degrees to rotate from `from` to `to`
// (positive = clockwise/right, negative = counter-clockwise/left).
inline int16_t headingDeltaDegrees(Heading from, Heading to) {
    int16_t diff = (static_cast<int16_t>(to) - static_cast<int16_t>(from)) * 90;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
}
