#pragma once
// ============================================================================
// config.h
//
// Single source of truth for pin assignments, physical dimensions, and
// tuning constants. Every value marked TODO must be verified/measured
// against the real robot and re-checked on competition day (bridge corner,
// start orientation, and finish location are announced on the day per the
// project brief).
//
// Nothing else in the codebase should hard-code a pin number or a physical
// constant -- always include config.h and reference the named constant.
// This keeps rewiring or re-tuning a one-file change.
// ============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Motor driver pins (one H-bridge module per motor, as supplied by the dept)
// ---------------------------------------------------------------------------
#define MOTOR_L_IN1   22
#define MOTOR_L_IN2   23
#define MOTOR_L_PWM   4      // must be a PWM-capable pin (Mega: 2-13, 44-46)

#define MOTOR_R_IN1   25
#define MOTOR_R_IN2   26
#define MOTOR_R_PWM   5      // must be a PWM-capable pin

// ---------------------------------------------------------------------------
// Quadrature encoder pins
// Mega external-interrupt-capable pins are limited to: 2, 3, 18, 19, 20, 21
// Channel A of each encoder MUST be on one of those pins; channel B can be
// any free digital pin and is only polled (no interrupt needed).
// ---------------------------------------------------------------------------
#define ENCODER_L_A   18
#define ENCODER_L_B   30
#define ENCODER_R_A   19
#define ENCODER_R_B   31

#define ENCODER_PULSES_PER_REV   330      // TODO: verify vs. gearbox+encoder datasheet
#define WHEEL_DIAMETER_MM        65.0f    // TODO: measure actual wheel
#define WHEEL_TRACK_MM           140.0f   // TODO: measure center-to-center wheel distance

// ---------------------------------------------------------------------------
// Ultrasonic sensors (wall detection: front / left / right)
// ---------------------------------------------------------------------------
#define US_FRONT_TRIG   32
#define US_FRONT_ECHO   33
#define US_LEFT_TRIG    34
#define US_LEFT_ECHO    35
#define US_RIGHT_TRIG   36
#define US_RIGHT_ECHO   37

#define US_MAX_RANGE_CM        150.0f
#define US_WALL_THRESHOLD_CM   13.0f  // TODO: calibrate -- distance below which a wall is "present"

// ---------------------------------------------------------------------------
// 8-channel IR line/floor sensor array (analog reflectance)
// ---------------------------------------------------------------------------
#define LINE_SENSOR_COUNT   8
// Outer-to-outer index order: 0 = far left ... 7 = far right
static const uint8_t LINE_SENSOR_PINS[LINE_SENSOR_COUNT] = {A8, A9, A10, A11, A12, A13, A14, A15};

#define LINE_BLACK_THRESHOLD   500   // TODO: calibrate against the actual floor/line/wall paint

// ---------------------------------------------------------------------------
// MPU-6050 gyroscope/accelerometer (I2C: Mega hardware SDA=20, SCL=21)
// ---------------------------------------------------------------------------
#define MPU6050_I2C_ADDR   0x68

// ---------------------------------------------------------------------------
// Misc I/O
// ---------------------------------------------------------------------------
#define START_BUTTON_PIN   38   // officials press this (or your own switch) to start timing
#define STATUS_LED_PIN     13

// Hold the existing start button while booting for this long to erase an old
// learned maze before a *new* arena is used.  A normal official start press
// is much shorter and never clears the map.
#define CLEAR_SAVED_MAP_HOLD_MS  2500UL

// ---------------------------------------------------------------------------
// Maze / arena geometry (from the project brief)
// ---------------------------------------------------------------------------
#define SECTION_A_WIDTH    4
#define SECTION_A_HEIGHT   4
#define SECTION_B_WIDTH    9
#define SECTION_B_HEIGHT   9

#define CELL_SIZE_MM       250.0f
#define WALL_HEIGHT_MM     100.0f

// ---------------------------------------------------------------------------
// Control tuning -- sensible starting points ONLY. Retune on the real field;
// surface friction and lighting are explicitly not guaranteed by the brief.
// ---------------------------------------------------------------------------
#define BASE_SPEED_PWM    150   // 0-255, forward driving speed
#define TURN_SPEED_PWM    120   // 0-255, in-place turning speed
#define MAX_SPEED_PWM     255

// Gyro-based heading-hold PID (keeps the robot driving straight / executes turns)
#define HEADING_KP   2.2f
#define HEADING_KI   0.0f
#define HEADING_KD   0.8f
#define HEADING_TOLERANCE_DEG   2.0f

// Small lateral-centering correction when both side sensors see corridor
// walls.  This supplements gyro heading-hold; tune it only after the basic
// straight-drive PID is stable.
#define WALL_CENTER_KP                 2.0f
#define WALL_CENTER_MAX_CORRECTION     25.0f
#define WALL_CENTER_SAMPLE_INTERVAL_MS 50UL

// Line-following PID (used only while crossing the bridge / line corridor)
#define LINE_KP   0.09f
#define LINE_KI   0.0f
#define LINE_KD   0.04f

// Safety cap so a bug can't spin the exploration loop forever
#define MAX_EXPLORE_STEPS   400
#define GOAL_CONFIRM_SAMPLES 3
#define GOAL_CONFIRM_SAMPLE_DELAY_MS 5
