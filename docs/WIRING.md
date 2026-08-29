# Wiring Reference

Mirrors `include/config.h`, which remains the source of truth — update
both if you rewire anything.

## Motor drivers (one H-bridge module per motor)

| Signal | Pin | Notes |
|---|---|---|
| Left IN1 | 22 | |
| Left IN2 | 23 | |
| Left PWM | 4  | must be PWM-capable |
| Right IN1 | 25 | |
| Right IN2 | 26 | |
| Right PWM | 5  | must be PWM-capable |

## Encoders (quadrature)

| Signal | Pin | Notes |
|---|---|---|
| Left A | 18 | Mega interrupt pin |
| Left B | 30 | polled, no interrupt needed |
| Right A | 19 | Mega interrupt pin |
| Right B | 31 | polled, no interrupt needed |

Mega interrupt-capable pins are limited to 2, 3, 18, 19, 20, 21 — channel A
of each encoder must stay on one of those.

## Ultrasonic sensors (HC-SR04 style: front / left / right wall detection)

| Signal | Pin |
|---|---|
| Front TRIG | 32 |
| Front ECHO | 33 |
| Left TRIG | 34 |
| Left ECHO | 35 |
| Right TRIG | 36 |
| Right ECHO | 37 |

## 8-channel IR line/floor sensor array (analog)

| Channel | Pin | Position |
|---|---|---|
| 0 | A8  | far left |
| 1 | A9  | |
| 2 | A10 | |
| 3 | A11 | center-left |
| 4 | A12 | center-right |
| 5 | A13 | |
| 6 | A14 | |
| 7 | A15 | far right |

## MPU-6050 gyroscope/accelerometer (I2C)

| Signal | Pin | Notes |
|---|---|---|
| SDA | 20 | Mega hardware I2C |
| SCL | 21 | Mega hardware I2C |
| ADDR | — | default address 0x68 |

## Misc

| Signal | Pin | Notes |
|---|---|---|
| Start button | 38 | INPUT_PULLUP, active-low |
| Status LED | 13 | onboard Mega LED |

## Calibration checklist (do this before the first real run)

1. Measure `WHEEL_DIAMETER_MM` and `WHEEL_TRACK_MM` with calipers, update `config.h`.
2. Confirm `ENCODER_PULSES_PER_REV` against your motor/encoder datasheet.
3. Place the robot in a cell with a known wall and tune `US_WALL_THRESHOLD_CM`
   until wall/no-wall detection is reliable at your approach distance.
4. Run the robot over the maze floor, the wall paint, and the bridge line
   and tune `LINE_BLACK_THRESHOLD` so all three are unambiguous.
5. Keep the robot still while it boots (the gyro auto-calibrates its zero
   offset in `GyroMPU6050::begin()`); then tune `HEADING_KP/KI/KD` for
   clean 90° turns and straight-line driving.
6. Tune `LINE_KP/KI/KD` on the actual bridge for smooth line tracking
   through its turns.
7. On competition day, set the announced bridge corner / start orientation
   in `lib/RobotFSM/RobotFSM.h` (`START_A_*`, `START_B_*`).
