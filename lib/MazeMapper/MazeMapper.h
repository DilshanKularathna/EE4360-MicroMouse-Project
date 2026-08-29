#pragma once
// ============================================================================
// MazeMapper
//
// Pure grid/graph logic for a rectangular maze: wall bookkeeping, visited
// tracking, and a classic micromouse-style flood-fill to compute shortest
// distance to a goal cell. Used for BOTH Section A (4x4) and Section B (9x9)
// -- just construct one instance per section.
//
// Intentionally hardware-free (no Arduino.h) so it can be unit tested on a
// host PC with `pio test -e native`. All robot movement/sensing lives in
// the Navigator class instead.
// ============================================================================

#include "RobotTypes.h"

class MazeMapper {
public:
    MazeMapper(uint8_t width, uint8_t height);
    ~MazeMapper();

    // Non-copyable (owns raw heap arrays) -- pass by reference instead.
    MazeMapper(const MazeMapper &) = delete;
    MazeMapper &operator=(const MazeMapper &) = delete;

    uint8_t width() const { return _w; }
    uint8_t height() const { return _h; }
    bool inBounds(int16_t x, int16_t y) const;

    // Marks a wall present/absent on one side of (x,y) AND mirrors it onto
    // the neighboring cell's opposite side, since a wall is shared by two
    // cells. Safe to call even if the neighbor is out of bounds.
    void setWall(uint8_t x, uint8_t y, WallDir dir, bool present);
    bool hasWall(uint8_t x, uint8_t y, WallDir dir) const;

    void markVisited(uint8_t x, uint8_t y);
    bool isVisited(uint8_t x, uint8_t y) const;

    void setGoal(uint8_t x, uint8_t y);
    bool hasGoal() const { return _goalSet; }
    Cell getGoal() const { return Cell{static_cast<int8_t>(_goalX), static_cast<int8_t>(_goalY)}; }

    // Recomputes BFS distance-to-goal for every reachable cell using only
    // walls that have been recorded so far. Unreachable/unknown cells are
    // left at UNREACHABLE. Call this once after setGoal() and again any
    // time new walls are learned before re-deriving a path.
    void floodFill();
    static const uint16_t UNREACHABLE = 0xFFFF;
    uint16_t getDistance(uint8_t x, uint8_t y) const;

    // Among the (at most 4) neighbors of (x,y) that are NOT blocked by a
    // known wall, returns the heading toward the one with the smallest
    // flood-fill distance. Requires floodFill() to have been run already.
    // Returns false if no legal, strictly-closer neighbor exists.
    bool getBestHeadingTowardGoal(uint8_t x, uint8_t y, Heading &outHeading) const;

private:
    uint8_t _w, _h;
    uint8_t *_walls;      // bitmask (WALL_NORTH|EAST|SOUTH|WEST) per cell
    bool *_visited;
    uint16_t *_dist;      // flood-fill distance per cell
    uint8_t _goalX, _goalY;
    bool _goalSet;

    uint16_t idx(uint8_t x, uint8_t y) const { return static_cast<uint16_t>(y) * _w + x; }
};
