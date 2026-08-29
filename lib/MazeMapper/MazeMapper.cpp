#include "MazeMapper.h"

MazeMapper::MazeMapper(uint8_t width, uint8_t height)
    : _w(width), _h(height), _goalX(0), _goalY(0), _goalSet(false) {
    uint16_t n = static_cast<uint16_t>(_w) * _h;
    _walls = new uint8_t[n];
    _visited = new bool[n];
    _dist = new uint16_t[n];
    for (uint16_t i = 0; i < n; i++) {
        _walls[i] = 0;
        _visited[i] = false;
        _dist[i] = UNREACHABLE;
    }
}

MazeMapper::~MazeMapper() {
    delete[] _walls;
    delete[] _visited;
    delete[] _dist;
}

void MazeMapper::clear() {
    uint16_t n = static_cast<uint16_t>(_w) * _h;
    for (uint16_t i = 0; i < n; i++) {
        _walls[i] = 0;
        _visited[i] = false;
        _dist[i] = UNREACHABLE;
    }
    _goalX = 0;
    _goalY = 0;
    _goalSet = false;
}

bool MazeMapper::inBounds(int16_t x, int16_t y) const {
    return x >= 0 && y >= 0 && x < _w && y < _h;
}

void MazeMapper::setWall(uint8_t x, uint8_t y, WallDir dir, bool present) {
    if (!inBounds(x, y)) return;

    if (present) _walls[idx(x, y)] |= dir;
    else _walls[idx(x, y)] &= ~dir;

    // Mirror onto the neighboring cell's opposite wall so the map stays
    // consistent regardless of which cell/sensor first observed it.
    Heading h = (dir == WALL_NORTH) ? Heading::NORTH
              : (dir == WALL_EAST)  ? Heading::EAST
              : (dir == WALL_SOUTH) ? Heading::SOUTH
              : Heading::WEST;
    Cell nb = neighborInDirection(Cell{static_cast<int8_t>(x), static_cast<int8_t>(y)}, h);
    if (inBounds(nb.x, nb.y)) {
        WallDir oppDir = headingToWallDir(oppositeOf(h));
        if (present) _walls[idx(nb.x, nb.y)] |= oppDir;
        else _walls[idx(nb.x, nb.y)] &= ~oppDir;
    }
}

bool MazeMapper::hasWall(uint8_t x, uint8_t y, WallDir dir) const {
    if (!inBounds(x, y)) return true; // treat out-of-bounds as walled off
    return (_walls[idx(x, y)] & dir) != 0;
}

uint8_t MazeMapper::getWallMask(uint8_t x, uint8_t y) const {
    if (!inBounds(x, y)) return WALL_NORTH | WALL_EAST | WALL_SOUTH | WALL_WEST;
    return _walls[idx(x, y)];
}

void MazeMapper::setWallMask(uint8_t x, uint8_t y, uint8_t mask) {
    if (!inBounds(x, y)) return;
    _walls[idx(x, y)] = mask & (WALL_NORTH | WALL_EAST | WALL_SOUTH | WALL_WEST);
}

void MazeMapper::markVisited(uint8_t x, uint8_t y) {
    if (inBounds(x, y)) _visited[idx(x, y)] = true;
}

bool MazeMapper::isVisited(uint8_t x, uint8_t y) const {
    if (!inBounds(x, y)) return false;
    return _visited[idx(x, y)];
}

void MazeMapper::setGoal(uint8_t x, uint8_t y) {
    _goalX = x;
    _goalY = y;
    _goalSet = true;
}

uint16_t MazeMapper::getDistance(uint8_t x, uint8_t y) const {
    if (!inBounds(x, y)) return UNREACHABLE;
    return _dist[idx(x, y)];
}

void MazeMapper::floodFill() {
    uint16_t n = static_cast<uint16_t>(_w) * _h;
    for (uint16_t i = 0; i < n; i++) _dist[i] = UNREACHABLE;
    if (!_goalSet) return;

    // Simple array-backed BFS queue -- the maze is at most 9x9 = 81 cells,
    // so a fixed-size queue sized to the whole grid is always sufficient.
    Cell *queue = new Cell[n];
    uint16_t head = 0, tail = 0;

    queue[tail++] = Cell{static_cast<int8_t>(_goalX), static_cast<int8_t>(_goalY)};
    _dist[idx(_goalX, _goalY)] = 0;

    const Heading dirs[4] = {Heading::NORTH, Heading::EAST, Heading::SOUTH, Heading::WEST};

    while (head < tail) {
        Cell cur = queue[head++];
        uint16_t curDist = _dist[idx(cur.x, cur.y)];

        for (uint8_t i = 0; i < 4; i++) {
            Heading h = dirs[i];
            // A move from `cur` to its neighbor in the OPPOSITE direction of
            // travel corresponds to checking the wall on `cur` in direction
            // `h`; since walls are mirrored, checking from `cur` is enough.
            if (hasWall(cur.x, cur.y, headingToWallDir(h))) continue;

            Cell nb = neighborInDirection(cur, h);
            if (!inBounds(nb.x, nb.y)) continue;
            if (_dist[idx(nb.x, nb.y)] != UNREACHABLE) continue;

            _dist[idx(nb.x, nb.y)] = curDist + 1;
            queue[tail++] = nb;
        }
    }

    delete[] queue;
}

bool MazeMapper::getBestHeadingTowardGoal(uint8_t x, uint8_t y, Heading &outHeading) const {
    uint16_t here = getDistance(x, y);
    if (here == UNREACHABLE) return false;

    const Heading dirs[4] = {Heading::NORTH, Heading::EAST, Heading::SOUTH, Heading::WEST};
    bool found = false;
    uint16_t best = here;

    for (uint8_t i = 0; i < 4; i++) {
        Heading h = dirs[i];
        if (hasWall(x, y, headingToWallDir(h))) continue;

        Cell nb = neighborInDirection(Cell{static_cast<int8_t>(x), static_cast<int8_t>(y)}, h);
        if (!inBounds(nb.x, nb.y)) continue;

        uint16_t d = getDistance(nb.x, nb.y);
        if (d != UNREACHABLE && d < best) {
            best = d;
            outHeading = h;
            found = true;
        }
    }
    return found;
}
