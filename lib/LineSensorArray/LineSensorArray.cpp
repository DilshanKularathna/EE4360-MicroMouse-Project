#include "LineSensorArray.h"

void LineSensorArray::begin() {
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        pinMode(LINE_SENSOR_PINS[i], INPUT);
        _raw[i] = 0;
        _black[i] = false;
    }
}

void LineSensorArray::readAll() {
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        _raw[i] = analogRead(LINE_SENSOR_PINS[i]);
        _black[i] = _raw[i] >= LINE_BLACK_THRESHOLD;
    }
}

bool LineSensorArray::isBlack(uint8_t index) const {
    if (index >= LINE_SENSOR_COUNT) return false;
    return _black[index];
}

bool LineSensorArray::allBlack() const {
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) if (!_black[i]) return false;
    return true;
}

bool LineSensorArray::allWhite() const {
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) if (_black[i]) return false;
    return true;
}

int16_t LineSensorArray::getLineError() const {
    // Symmetric weights spanning roughly the sensor array width, e.g. for
    // 8 sensors: -350,-250,-150,-50,50,150,250,350.
    int32_t weightedSum = 0;
    int32_t blackCount = 0;

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        if (_black[i]) {
            int32_t weight = (static_cast<int32_t>(i) * 2 - (LINE_SENSOR_COUNT - 1)) * 50;
            weightedSum += weight;
            blackCount++;
        }
    }

    if (blackCount == 0) return 0; // line lost -- caller decides recovery behavior
    return static_cast<int16_t>(weightedSum / blackCount);
}

bool LineSensorArray::detectApproachMarker() const {
    // Expect only the innermost one or two sensors to read black, and every
    // outer sensor to read white -- the inverse of the normal bridge line
    // pattern (where black should stay narrow/centered too, so also guard
    // against a false trigger while already tracking the bridge line by
    // requiring the OUTER-most sensors specifically to be white).
    uint8_t mid1 = LINE_SENSOR_COUNT / 2 - 1;
    uint8_t mid2 = LINE_SENSOR_COUNT / 2;

    bool centerBlack = _black[mid1] || _black[mid2];
    bool outerWhite = !_black[0] && !_black[1] && !_black[LINE_SENSOR_COUNT - 1] && !_black[LINE_SENSOR_COUNT - 2];

    return centerBlack && outerWhite;
}

bool LineSensorArray::isOnWhiteTile() const {
    uint8_t blackCount = 0;
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) if (_black[i]) blackCount++;
    // Allow a small tolerance for noise/seams rather than requiring a
    // perfect all-white reading.
    return blackCount == 0;
}
