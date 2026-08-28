# MazeSolverBot

PlatformIO project template for the EE4360 Maze Solver Challenge Project 2026.

## Structure

- `platformio.ini` - hardware env (`megaatmega2560`) + native test env
- `include/config.h` - single source of truth: all pins + tuning constants
- `include/RobotTypes.h` - hardware-free shared types (Heading, Cell, etc.)
- `lib/MotorDriver/` - H-bridge wrapper
- `lib/Encoder/` - quadrature pulse counting (interrupt-driven)
- `lib/UltrasonicSensor/` - HC-SR04 style wall ranging
- `lib/LineSensorArray/` - 8ch IR: line-following error + floor-pattern detection
- `lib/GyroMPU6050/` - register-level I2C driver, no external lib needed
- `lib/PIDController/` - generic PID reused for heading-hold AND line-following
- `lib/MazeMapper/` - pure flood-fill algorithm (no Arduino dependency)
- `lib/Navigator/` - movement primitives + DFS exploration + shortest-path run
- `lib/BridgeHandler/` - line-following across the bridge/corridor
- `lib/RobotFSM/` - top-level mission state machine
- `src/main.cpp` - composition root only
- `test/test_maze_mapper/` - Unity unit tests for the flood-fill logic

## Build

```
pio run -e megaatmega2560       # build for hardware
pio run -e megaatmega2560 -t upload
pio test -e native              # run MazeMapper unit tests on host
```
