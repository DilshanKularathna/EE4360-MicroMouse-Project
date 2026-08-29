#include "MazeRobotFSM.h"

namespace {

// Plain free functions used as goal-detector callbacks for
// MazeNavigator::exploreAndFindGoal (see MazeNavigator.h for why this is a raw
// function pointer rather than std::function).
bool isBridgeApproachMarker(MazeLineSensorArray &lineSensors) {
    lineSensors.readAll();
    return lineSensors.detectApproachMarker();
}

bool isFinishTile(MazeLineSensorArray &lineSensors) {
    lineSensors.readAll();
    return lineSensors.isOnWhiteTile();
}

} // namespace

MazeRobotFSM::MazeRobotFSM(MazeNavigator &navigator, MazeBridgeHandler &bridge, MazeLineSensorArray &lineSensors)
    : _navigator(navigator), _bridge(bridge), _lineSensors(lineSensors),
      _mazeA(SECTION_A_WIDTH, SECTION_A_HEIGHT),
      _mazeB(SECTION_B_WIDTH, SECTION_B_HEIGHT),
      _state(RobotState::INIT) {}

void MazeRobotFSM::run() {
    setState(RobotState::WAIT_FOR_START);
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    while (digitalRead(START_BUTTON_PIN) == HIGH) {
        // Idle until the official presses the start button. Timing (per
        // the brief) begins the moment the robot's front edge leaves the
        // START tile, which happens naturally once driveOneCellForward()
        // is first called below.
    }

    // ---------------- Phase 1: Section A -> find the bridge exit --------
    setState(RobotState::SECTION_A_EXPLORE);
    uint8_t goalAx = 0, goalAy = 0;
    Heading finalAHeading = START_A_HEADING;

    bool foundBridge = _navigator.exploreAndFindGoal(
        _mazeA, START_A_X, START_A_Y, START_A_HEADING,
        _lineSensors, isBridgeApproachMarker,
        goalAx, goalAy, finalAHeading);

    if (!foundBridge) {
        // Exploration exhausted the reachable map without seeing the
        // marker -- something is miscalibrated (sensor threshold, wiring)
        // or the marker heuristic needs retuning against the real field.
        setState(RobotState::ERROR);
        return;
    }

    // Re-run Section A via the shortest known path (path-optimization
    // bonus) before actually committing to crossing the bridge.
    setState(RobotState::SECTION_A_SPEED_RUN);
    _navigator.runShortestPath(_mazeA, START_A_X, START_A_Y, START_A_HEADING, goalAx, goalAy);

    // ---------------- Phase 2: cross the bridge --------------------------
    setState(RobotState::BRIDGE_CROSSING);
    bool crossed = _bridge.crossBridge();
    if (!crossed) {
        setState(RobotState::ERROR);
        return;
    }

    // ---------------- Phase 3: Section B -> find the finish tile --------
    setState(RobotState::SECTION_B_EXPLORE);
    uint8_t goalBx = 0, goalBy = 0;
    Heading finalBHeading = START_B_HEADING;

    bool foundFinish = _navigator.exploreAndFindGoal(
        _mazeB, START_B_X, START_B_Y, START_B_HEADING,
        _lineSensors, isFinishTile,
        goalBx, goalBy, finalBHeading);

    if (!foundFinish) {
        setState(RobotState::ERROR);
        return;
    }

    // ---------------- Phase 4: speed run to the finish -------------------
    setState(RobotState::SECTION_B_SPEED_RUN);
    _navigator.runShortestPath(_mazeB, START_B_X, START_B_Y, START_B_HEADING, goalBx, goalBy);

    setState(RobotState::FINISHED);
    _navigator.stopMotors();
}
