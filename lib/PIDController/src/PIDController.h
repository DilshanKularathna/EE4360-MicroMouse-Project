#pragma once
// PIDController
// Generic, hardware-agnostic PID controller.
// Reused for:
//  - heading-hold (gyro error -> motor correction)
//  - line-following (line sensor error -> motor correction)

class PIDController {
public:
    // TODO: constructor(kp, ki, kd), compute(setpoint, measured), reset()
};
