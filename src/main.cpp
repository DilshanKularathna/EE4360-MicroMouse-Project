// ============================================================================
// main.cpp
//
// Composition root only: construct every module, wire them together, hand
// off to RobotFSM. No maze-solving or control logic lives here on purpose
// -- that keeps this file short and makes every subsystem independently
// reusable/testable (see lib/*, and test/test_maze_mapper for an example
// of testing MazeMapper without any hardware at all).
// ============================================================================

#include <Arduino.h>
#include "config.h"

#include "MotorDriver.h"
#include "Encoder.h"
#include "UltrasonicSensor.h"
#include "LineSensorArray.h"
#include "GyroMPU6050.h"
#include "Navigator.h"
#include "BridgeHandler.h"
#include "RobotFSM.h"

// --- Actuators ---------------------------------------------------------
MotorDriver leftMotor(MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_PWM);
MotorDriver rightMotor(MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_PWM);

// --- Sensors -------------------------------------------------------------
Encoder leftEncoder(ENCODER_L_A, ENCODER_L_B);
Encoder rightEncoder(ENCODER_R_A, ENCODER_R_B);

UltrasonicSensor frontUs(US_FRONT_TRIG, US_FRONT_ECHO, US_MAX_RANGE_CM);
UltrasonicSensor leftUs(US_LEFT_TRIG, US_LEFT_ECHO, US_MAX_RANGE_CM);
UltrasonicSensor rightUs(US_RIGHT_TRIG, US_RIGHT_ECHO, US_MAX_RANGE_CM);

LineSensorArray lineSensors;
GyroMPU6050 gyro;

// --- Behavior layers -----------------------------------------------------
Navigator navigator(leftMotor, rightMotor, leftEncoder, rightEncoder, frontUs, leftUs, rightUs, gyro);
BridgeHandler bridge(leftMotor, rightMotor, lineSensors);
RobotFSM robot(navigator, bridge, lineSensors);

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED_PIN, OUTPUT);

    navigator.begin();
    leftEncoder.begin(0);
    rightEncoder.begin(1);
    frontUs.begin();
    leftUs.begin();
    rightUs.begin();
    lineSensors.begin();

    Serial.println(F("Initializing MPU-6050... keep the robot still."));
    if (!gyro.begin()) {
        Serial.println(F("ERROR: MPU-6050 not responding. Check I2C wiring (SDA=20, SCL=21)."));
        while (true) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
            delay(150); // fast blink = hardware init failure
        }
    }

    Serial.println(F("Ready. Press the start button to begin the run."));
    digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {
    robot.run();

    // run() only returns once (FINISHED or ERROR). Reflect the outcome on
    // the status LED; power-cycle or reset the Mega to attempt a new run.
    bool ok = (robot.getState() == RobotState::FINISHED);
    while (true) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(ok ? 800 : 100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(ok ? 800 : 100);
    }
}
