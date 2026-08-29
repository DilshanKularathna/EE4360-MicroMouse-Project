// ============================================================================
// Native (no-hardware) unit tests for MazeMapper.
//
// Run with:  pio test -e native
//
// This is worth having precisely because the flood-fill algorithm is the
// part most likely to have an off-by-one or wall-orientation bug, and it's
// far cheaper to catch that here than on the actual field with 30 minutes
// of arena access ticking away.
// ============================================================================

#include <unity.h>
#include "MazeMapper.h"

void test_open_grid_manhattan_distance() {
    // A 4x4 maze with no walls at all: distance to the goal should just be
    // Manhattan distance.
    MazeMapper maze(4, 4);
    maze.setGoal(3, 3);
    maze.floodFill();

    TEST_ASSERT_EQUAL_UINT16(0, maze.getDistance(3, 3));
    TEST_ASSERT_EQUAL_UINT16(6, maze.getDistance(0, 0)); // |3-0| + |3-0|
    TEST_ASSERT_EQUAL_UINT16(3, maze.getDistance(0, 3));
    TEST_ASSERT_EQUAL_UINT16(3, maze.getDistance(3, 0));
}

void test_wall_forces_detour() {
    // 3x2 grid: seal off the entire top row so the only route from (0,0)
    // to the goal at (2,0) is down through the bottom row and back up,
    // giving a true path length of 4 instead of the direct Manhattan
    // distance of 2.
    MazeMapper maze(3, 2);
    maze.setGoal(2, 0);

    maze.setWall(0, 0, WALL_EAST, true); // blocks (0,0)<->(1,0)
    maze.setWall(1, 0, WALL_EAST, true); // blocks (1,0)<->(2,0)

    maze.floodFill();

    uint16_t d = maze.getDistance(0, 0);
    TEST_ASSERT_TRUE(d > 2); // longer than the blocked direct Manhattan distance
    TEST_ASSERT_EQUAL_UINT16(4, d);
}

void test_best_heading_moves_closer() {
    MazeMapper maze(4, 4);
    maze.setGoal(3, 0);
    maze.floodFill();

    Heading h;
    bool ok = maze.getBestHeadingTowardGoal(0, 0, h);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Heading::EAST), static_cast<int>(h));
}

void test_unreachable_cell_reports_unreachable() {
    MazeMapper maze(2, 2);
    maze.setGoal(1, 1);

    // Wall the goal cell in completely.
    maze.setWall(1, 1, WALL_NORTH, true);
    maze.setWall(1, 1, WALL_WEST, true);

    maze.floodFill();

    TEST_ASSERT_EQUAL_UINT16(MazeMapper::UNREACHABLE, maze.getDistance(0, 0));
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_open_grid_manhattan_distance);
    RUN_TEST(test_wall_forces_detour);
    RUN_TEST(test_best_heading_moves_closer);
    RUN_TEST(test_unreachable_cell_reports_unreachable);
    return UNITY_END();
}
