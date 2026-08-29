#include "GyroMPU6050.h"
#include <Wire.h>
#include "config.h"

// MPU-6050 register map (only what we need)
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_GYRO_ZOUT_H  = 0x47;

// At the default +/-250 deg/s full-scale range, sensitivity is 131 LSB/(deg/s).
static const float GYRO_SENSITIVITY_LSB_PER_DEG_S = 131.0f;
static const float ACCEL_SENSITIVITY_LSB_PER_G = 16384.0f; // default +/-2g range

void GyroMPU6050::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

int16_t GyroMPU6050::readWord(uint8_t regHigh) {
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(regHigh);
    Wire.endTransmission(false);
    Wire.requestFrom(static_cast<int>(MPU6050_I2C_ADDR), 2, true);
    int16_t value = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    return value;
}

bool GyroMPU6050::begin() {
    Wire.begin();

    Wire.beginTransmission(MPU6050_I2C_ADDR);
    uint8_t whoAmIStatus = Wire.endTransmission();
    if (whoAmIStatus != 0) return false; // device not responding

    writeRegister(REG_PWR_MGMT_1, 0x00);   // wake the device up
    writeRegister(REG_GYRO_CONFIG, 0x00);  // +/-250 deg/s full scale

    delay(50); // let the sensor settle before calibrating
    calibrateGyroZOffset();

    _lastUpdateUs = micros();
    return true;
}

void GyroMPU6050::calibrateGyroZOffset() {
    // Robot MUST be held still during begin() for this to be valid.
    const int samples = 200;
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += readWord(REG_GYRO_ZOUT_H);
        delay(2);
    }
    _gyroZOffsetDegPerSec = (static_cast<float>(sum) / samples) / GYRO_SENSITIVITY_LSB_PER_DEG_S;
}

void GyroMPU6050::update() {
    unsigned long nowUs = micros();
    float dtSec = (nowUs - _lastUpdateUs) / 1000000.0f;
    _lastUpdateUs = nowUs;

    int16_t rawGz = readWord(REG_GYRO_ZOUT_H);
    float gyroZDegPerSec = (rawGz / GYRO_SENSITIVITY_LSB_PER_DEG_S) - _gyroZOffsetDegPerSec;
    _headingDeg += gyroZDegPerSec * dtSec;

    // Keep heading wrapped to (-180, 180] for predictable PID error math.
    while (_headingDeg > 180.0f) _headingDeg -= 360.0f;
    while (_headingDeg <= -180.0f) _headingDeg += 360.0f;

    int16_t rawAx = readWord(REG_ACCEL_XOUT_H);
    int16_t rawAy = readWord(static_cast<uint8_t>(REG_ACCEL_XOUT_H + 2));
    int16_t rawAz = readWord(static_cast<uint8_t>(REG_ACCEL_XOUT_H + 4));
    _accelXg = rawAx / ACCEL_SENSITIVITY_LSB_PER_G;
    _accelYg = rawAy / ACCEL_SENSITIVITY_LSB_PER_G;
    _accelZg = rawAz / ACCEL_SENSITIVITY_LSB_PER_G;
}
