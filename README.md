# Maze Solver Bot — EE4360 Maze Solver Challenge 2026

Firmware for an Arduino Mega-based autonomous maze-solving robot: explores
a 4×4 arena (Section A), finds and crosses a line-guided bridge, then
explores a 9×9 arena (Section B) to reach the finish tile — re-running the
shortest known path in both sections once it's been discovered.

This README covers **how the project is organized and how to build/test
it.** It does not repeat the competition rules — see the original project
brief PDF for those.

## Quick start

```bash
pio run                     # build firmware
pio run -t upload           # flash the Arduino Mega
pio device monitor          # serial monitor, 115200 baud
pio test -e native          # run the algorithm unit tests on your PC (no robot needed)
```

## Why the project is laid out this way

Every hardware component gets its **own library folder** under `lib/`,
each with a `.h`/`.cpp` pair that only knows about its own piece of
hardware (one motor, one encoder, one ultrasonic sensor, the IR array, the
gyro). None of them know anything about mazes, states, or strategy. That
split matters for three reasons that map directly onto how this project
gets graded and defended:

1. **Readability.** Opening `lib/MotorDriver/MotorDriver.cpp` tells you
   everything about driving one motor and nothing else. You never have to
   read encoder or maze-solving logic to understand motor control.
2. **Reusability.** `MazeMapper` and `Navigator` don't hard-code "Section A"
   or "Section B" anywhere — they take a maze and a starting position as
   parameters. The exact same code explores and speed-runs both sections;
   only the constants passed in from `RobotFSM` differ. If a future year's
   challenge changes the grid to 6×6, nothing in `lib/` needs to change.
3. **Viva readiness.** The brief specifically asks each student to explain
   *why* the code is structured the way it is, and to answer hardware
   questions tied to specific modules. A clean module boundary makes both
   much easier to defend, and makes it obvious (and provable via git
   blame/commit history) who wrote which subsystem.

### Layer summary

| Layer | Location | Knows about hardware? | Knows about the maze? |
|---|---|---|---|
| Drivers | `lib/MotorDriver`, `lib/Encoder`, `lib/UltrasonicSensor`, `lib/LineSensorArray`, `lib/GyroMPU6050` | Yes (one pin/sensor each) | No |
| Control | `lib/PIDController` | No | No |
| Algorithm | `lib/MazeMapper` | No | Yes (walls, flood-fill) |
| Behavior | `lib/Navigator`, `lib/BridgeHandler` | Yes (via driver references) | Yes (via `MazeMapper&`) |
| Orchestration | `lib/RobotFSM` | No (delegates) | No (delegates) |
| Composition | `src/main.cpp` | Wires everything together | — |

`MazeMapper` is deliberately the *only* file with zero Arduino dependency
(`#include <Arduino.h>` never appears in it), which is what lets it run
in `pio test -e native` on your laptop — see below.

## Testing the maze-solving algorithm without a robot

`test/test_maze_mapper/test_main.cpp` runs the flood-fill algorithm
against known mazes (open grid, a forced detour, an unreachable goal) and
checks the distances/directions it produces against hand-calculated
answers. This is the single most bug-prone piece of logic in the whole
project — an off-by-one in wall orientation silently sends the real robot
into a wall — so it's worth running (`pio test -e native`) any time you
touch `MazeMapper`, before ever putting the robot on the field.

## What's implemented vs. what you still need to tune

**Implemented and verified** (compiled against the real Arduino Mega core
and linked into a working firmware image; the flood-fill algorithm was
additionally checked against hand-computed expected results):
- Motor/encoder/ultrasonic/line-array/gyro drivers
- Generic PID controller
- BFS flood-fill shortest-path algorithm
- DFS-with-backtrack exploration
- Gyro-held straight driving and 90° turns
- PID line-following for the bridge
- The full mission state machine end to end

**You still need to, before this wins any races:**
- Measure and set `WHEEL_DIAMETER_MM`, `WHEEL_TRACK_MM`,
  `ENCODER_PULSES_PER_REV` in `include/config.h` against your actual
  chassis and motors.
- Calibrate `US_WALL_THRESHOLD_CM` and `LINE_BLACK_THRESHOLD` against the
  real maze surfaces and lighting (the brief explicitly warns lighting
  isn't guaranteed).
- Tune `HEADING_KP/KI/KD` and `LINE_KP/KI/KD` on the actual field —
  starting values in `config.h` are reasonable defaults, not measured ones.
- Set `START_A_X/Y`, `START_B_X/Y`, and their headings in
  `lib/RobotFSM/RobotFSM.h` once the officials announce the bridge corner
  and orientation on competition day.
- `LineSensorArray::detectApproachMarker()` uses a simple center-vs-outer
  heuristic — verify it against the real marker tile at your approach
  speed and adjust which sensor indices count as "center."
- The DFS exploration strategy is a solid, understandable baseline but not
  the most efficient possible search; if you want a competitive edge,
  consider extending it to explore with flood-fill guidance instead of
  pure DFS backtracking (the flood-fill code for the *speed run* is
  already fully general and doesn't need to change).

## Hardware

See [`docs/WIRING.md`](docs/WIRING.md) for the full pin table (mirrors
`include/config.h` — that file is still the source of truth; the doc is
just a readable summary for the wiring bench and the viva).
