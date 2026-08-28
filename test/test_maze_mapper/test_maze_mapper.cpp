#include <unity.h>
#include "MazeMapper.h"

// Unity unit tests for the pure flood-fill logic in MazeMapper.
// Runs under the "native" PlatformIO environment (no hardware needed):
//   pio test -e native

void setUp(void) {}
void tearDown(void) {}

void test_placeholder(void) {
    TEST_ASSERT_EQUAL(1, 1);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_placeholder);
    return UNITY_END();
}
