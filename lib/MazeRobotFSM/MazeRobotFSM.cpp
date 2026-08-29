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
      _state(RobotState::INIT),
      _mode(RunMode::EXPLORE),
      _goalAX(0), _goalAY(0), _goalBX(0), _goalBY(0) {}

void MazeRobotFSM::clearSavedMapOnLongBootPress() {
    // A saved route is deliberately retained across reset for the next trial.
    // Holding the existing start button during boot is an explicit, physical
    // way to discard it before testing on a different maze; it requires no
    // reprogramming or additional hardware.
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(START_BUTTON_PIN) != LOW) return;

    unsigned long pressedAt = millis();
    while (digitalRead(START_BUTTON_PIN) == LOW) {
        if (millis() - pressedAt >= CLEAR_SAVED_MAP_HOLD_MS) {
            _store.clear();
            Serial.println(F("Saved maze cleared. Release the button, then start normally."));
            while (digitalRead(START_BUTTON_PIN) == LOW) {}
            return;
        }
    }
}

void MazeRobotFSM::waitForOfficialStart() {
    setState(RobotState::WAIT_FOR_START);
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    while (digitalRead(START_BUTTON_PIN) == HIGH) {
        // Idle until the official presses the start button. Timing (per
        // the brief) begins the moment the robot's front edge leaves the
        // START tile, which happens naturally once driveOneCellForward()
        // is first called below.
    }
}

void MazeRobotFSM::exploreCourse() {
    _navigator.setHeadingReference(START_A_HEADING);

    // ---------------- Phase 1: Section A -> find the bridge exit --------
    setState(RobotState::SECTION_A_EXPLORE);
    Heading finalAHeading = START_A_HEADING;

    bool foundBridge = _navigator.exploreAndFindGoal(
        _mazeA, START_A_X, START_A_Y, START_A_HEADING,
        _lineSensors, isBridgeApproachMarker,
        _goalAX, _goalAY, finalAHeading);

    if (!foundBridge) {
        // Exploration exhausted the reachable map without seeing the
        // marker -- something is miscalibrated (sensor threshold, wiring)
        // or the marker heuristic needs retuning against the real field.
        setState(RobotState::ERROR);
        return;
    }

    // exploreAndFindGoal() has now returned the robot to START_A, so this is
    // a real shortest-path drive rather than an impossible "teleport" back
    // to the start coordinates.
    setState(RobotState::SECTION_A_SPEED_RUN);
    _navigator.runShortestPath(_mazeA, START_A_X, START_A_Y, START_A_HEADING, _goalAX, _goalAY);

    // ---------------- Phase 2: cross the bridge --------------------------
    setState(RobotState::BRIDGE_CROSSING);
    bool crossed = _bridge.crossBridge();
    if (!crossed) {
        setState(RobotState::ERROR);
        return;
    }

    // The bridge corridor direction is announced on the day.  Once it has
    // been crossed, establish that known Section-B map-frame heading.
    _navigator.setHeadingReference(START_B_HEADING);

    // ---------------- Phase 3: Section B -> find the finish tile --------
    setState(RobotState::SECTION_B_EXPLORE);
    Heading finalBHeading = START_B_HEADING;

    bool foundFinish = _navigator.exploreAndFindGoal(
        _mazeB, START_B_X, START_B_Y, START_B_HEADING,
        _lineSensors, isFinishTile,
        _goalBX, _goalBY, finalBHeading);

    if (!foundFinish) {
        setState(RobotState::ERROR);
        return;
    }

    // The Section-B DFS has also returned to its entry/start cell.  Run its
    // discovered shortest path to FINISH, then retain both maps for a faster
    // complete attempt after a reset.
    setState(RobotState::SECTION_B_SPEED_RUN);
    _navigator.runShortestPath(_mazeB, START_B_X, START_B_Y, START_B_HEADING, _goalBX, _goalBY);

    _store.save(_mazeA, _goalAX, _goalAY, _mazeB, _goalBX, _goalBY);
    setState(RobotState::FINISHED);
    _navigator.stopMotors();
}

void MazeRobotFSM::runSavedCourse() {
    _navigator.setHeadingReference(START_A_HEADING);

    // ---------------- Known fastest route: START -> bridge -> FINISH -----
    setState(RobotState::SECTION_A_SPEED_RUN);
    _navigator.runShortestPath(_mazeA, START_A_X, START_A_Y, START_A_HEADING, _goalAX, _goalAY);

    setState(RobotState::BRIDGE_CROSSING);
    if (!_bridge.crossBridge()) {
        setState(RobotState::ERROR);
        return;
    }

    _navigator.setHeadingReference(START_B_HEADING);
    setState(RobotState::SECTION_B_SPEED_RUN);
    _navigator.runShortestPath(_mazeB, START_B_X, START_B_Y, START_B_HEADING, _goalBX, _goalBY);

    setState(RobotState::FINISHED);
    _navigator.stopMotors();
}

void MazeRobotFSM::run() {
    clearSavedMapOnLongBootPress();
    _mode = _store.load(_mazeA, _goalAX, _goalAY, _mazeB, _goalBX, _goalBY)
          ? RunMode::SPEED_RUN
          : RunMode::EXPLORE;

    if (_mode == RunMode::SPEED_RUN) {
        Serial.println(F("Saved maze found: next attempt is a speed run."));
    } else {
        Serial.println(F("No saved maze: next attempt will explore."));
    }

    waitForOfficialStart();
    if (_mode == RunMode::SPEED_RUN) runSavedCourse();
    else exploreCourse();
}
