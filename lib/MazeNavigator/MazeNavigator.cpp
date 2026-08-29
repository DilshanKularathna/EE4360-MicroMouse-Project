#include "MazeNavigator.h"

namespace {

float headingToAngleDeg(Heading h) {
    switch (h) {
        case Heading::NORTH: return 0.0f;
        case Heading::EAST:  return 90.0f;
        case Heading::SOUTH: return 180.0f;
        default:              return -90.0f; // WEST
    }
}

float wrapDeg180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

// Preference order when exploring: straight ahead first (keeps corridors
// fast to traverse), then right, then left, and only reverse as a last
// resort before the caller falls back to backtracking.
bool findUnvisitedNeighbor(MazeMapper &maze, Cell cur, Heading curHeading, Heading &outHeading) {
    Heading candidates[4] = {curHeading, turnRightOf(curHeading), turnLeftOf(curHeading), oppositeOf(curHeading)};
    for (uint8_t i = 0; i < 4; i++) {
        Heading h = candidates[i];
        if (maze.hasWall(cur.x, cur.y, headingToWallDir(h))) continue;

        Cell nb = neighborInDirection(cur, h);
        if (!maze.inBounds(nb.x, nb.y)) continue;
        if (maze.isVisited(nb.x, nb.y)) continue;

        outHeading = h;
        return true;
    }
    return false;
}

Heading directionBetween(Cell from, Cell to) {
    if (to.x == from.x && to.y == from.y - 1) return Heading::NORTH;
    if (to.x == from.x && to.y == from.y + 1) return Heading::SOUTH;
    if (to.x == from.x + 1 && to.y == from.y) return Heading::EAST;
    return Heading::WEST;
}

} // namespace

MazeNavigator::MazeNavigator(MazeMotorDriver &leftMotor, MazeMotorDriver &rightMotor,
                      MazeEncoder &leftEncoder, MazeEncoder &rightEncoder,
                      MazeUltrasonicSensor &frontUs, MazeUltrasonicSensor &leftUs, MazeUltrasonicSensor &rightUs,
                      MazeGyroMPU6050 &gyro)
    : _leftMotor(leftMotor), _rightMotor(rightMotor),
      _leftEncoder(leftEncoder), _rightEncoder(rightEncoder),
      _frontUs(frontUs), _leftUs(leftUs), _rightUs(rightUs),
      _gyro(gyro),
      _headingPid(HEADING_KP, HEADING_KI, HEADING_KD, -TURN_SPEED_PWM, TURN_SPEED_PWM) {}

void MazeNavigator::begin() {
    _leftMotor.begin();
    _rightMotor.begin();
    stopMotors();
}

void MazeNavigator::stopMotors() {
    _leftMotor.stop();
    _rightMotor.stop();
}

void MazeNavigator::turnToHeading(Heading target) {
    float targetDeg = headingToAngleDeg(target);
    _headingPid.reset();
    unsigned long lastMicros = micros();

    while (true) {
        _gyro.update();
        float error = wrapDeg180(targetDeg - _gyro.getHeadingDeg());
        if (fabs(error) <= HEADING_TOLERANCE_DEG) break;

        float dt = (micros() - lastMicros) / 1000000.0f;
        lastMicros = micros();
        float output = _headingPid.compute(error, dt);

        // In-place rotation: wheels spin in opposite directions.
        _leftMotor.setSpeed(static_cast<int16_t>(-output));
        _rightMotor.setSpeed(static_cast<int16_t>(output));
    }
    stopMotors();
}

void MazeNavigator::driveWithHeadingHold(float targetHeadingDeg, int16_t baseSpeed, long targetEncoderTicks) {
    _leftEncoder.reset();
    _rightEncoder.reset();
    _headingPid.reset();
    unsigned long lastMicros = micros();

    while (true) {
        _gyro.update();
        long leftTicks = labs(_leftEncoder.getCount());
        long rightTicks = labs(_rightEncoder.getCount());
        long avgTicks = (leftTicks + rightTicks) / 2;
        if (avgTicks >= targetEncoderTicks) break;

        float error = wrapDeg180(targetHeadingDeg - _gyro.getHeadingDeg());
        float dt = (micros() - lastMicros) / 1000000.0f;
        lastMicros = micros();
        float correction = _headingPid.compute(error, dt);

        _leftMotor.setSpeed(static_cast<int16_t>(baseSpeed - correction));
        _rightMotor.setSpeed(static_cast<int16_t>(baseSpeed + correction));
    }
    stopMotors();
}

void MazeNavigator::driveOneCellForward() {
    float holdHeadingDeg = _gyro.getHeadingDeg(); // hold whatever we're currently facing
    float circumferenceMM = PI * WHEEL_DIAMETER_MM;
    long targetTicks = static_cast<long>((CELL_SIZE_MM / circumferenceMM) * ENCODER_PULSES_PER_REV);
    driveWithHeadingHold(holdHeadingDeg, BASE_SPEED_PWM, targetTicks);
}

void MazeNavigator::senseWalls(MazeMapper &maze, uint8_t x, uint8_t y, Heading heading) {
    float frontCm = _frontUs.readDistanceCm();
    float leftCm  = _leftUs.readDistanceCm();
    float rightCm = _rightUs.readDistanceCm();

    maze.setWall(x, y, headingToWallDir(heading), frontCm < US_WALL_THRESHOLD_CM);
    maze.setWall(x, y, headingToWallDir(turnLeftOf(heading)), leftCm < US_WALL_THRESHOLD_CM);
    maze.setWall(x, y, headingToWallDir(turnRightOf(heading)), rightCm < US_WALL_THRESHOLD_CM);
}

bool MazeNavigator::exploreAndFindGoal(MazeMapper &maze, uint8_t startX, uint8_t startY, Heading startHeading,
                                    MazeLineSensorArray &lineSensors, GoalDetectorFn isGoalDetected,
                                    uint8_t &outGoalX, uint8_t &outGoalY, Heading &outFinalHeading) {
    uint16_t maxCells = static_cast<uint16_t>(maze.width()) * maze.height();
    Cell *stack = new Cell[maxCells];
    uint16_t stackTop = 0;

    Cell cur{static_cast<int8_t>(startX), static_cast<int8_t>(startY)};
    Heading heading = startHeading;

    maze.markVisited(cur.x, cur.y);
    stack[stackTop++] = cur;

    uint16_t steps = 0;
    bool found = false;

    while (stackTop > 0 && steps < MAX_EXPLORE_STEPS) {
        steps++;
        senseWalls(maze, cur.x, cur.y, heading);

        if (isGoalDetected(lineSensors)) {
            outGoalX = cur.x;
            outGoalY = cur.y;
            outFinalHeading = heading;
            found = true;
            break;
        }

        Heading nextDir;
        if (findUnvisitedNeighbor(maze, cur, heading, nextDir)) {
            turnToHeading(nextDir);
            driveOneCellForward();
            cur = neighborInDirection(cur, nextDir);
            heading = nextDir;
            maze.markVisited(cur.x, cur.y);
            stack[stackTop++] = cur;
        } else {
            // Dead end -- pop back to the previous cell and retry from there.
            stackTop--;
            if (stackTop == 0) break; // fully explored without finding the goal
            Cell prev = stack[stackTop - 1];
            Heading backDir = directionBetween(cur, prev);
            turnToHeading(backDir);
            driveOneCellForward();
            cur = prev;
            heading = backDir;
        }
    }

    delete[] stack;
    return found;
}

void MazeNavigator::runShortestPath(MazeMapper &maze, uint8_t startX, uint8_t startY, Heading startHeading,
                                 uint8_t goalX, uint8_t goalY) {
    (void)startHeading; // caller guarantees the robot is physically oriented this way already

    maze.setGoal(goalX, goalY);
    maze.floodFill();

    Cell cur{static_cast<int8_t>(startX), static_cast<int8_t>(startY)};

    while (!(cur.x == static_cast<int8_t>(goalX) && cur.y == static_cast<int8_t>(goalY))) {
        Heading nextDir;
        if (!maze.getBestHeadingTowardGoal(cur.x, cur.y, nextDir)) break; // map inconsistency safeguard

        turnToHeading(nextDir);
        driveOneCellForward();
        cur = neighborInDirection(cur, nextDir);
    }
}
