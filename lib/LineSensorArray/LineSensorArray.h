#pragma once
// ============================================================================
// LineSensorArray
//
// Wraps the 8-channel downward-facing IR reflectance array. Serves two
// distinct jobs described in the brief:
//   1. Line-following error signal while crossing the bridge / line corridor
//      (dark line on a light/white background).
//   2. Floor-pattern recognition used as "goal detected" signals during
//      maze exploration:
//        - detectApproachMarker(): the 9cm white / 3cm black bridge-approach
//          marker tile (Figure 2 in the brief) -> tells Navigator it has
//          found the bridge exit in Section A.
//        - isOnWhiteTile(): a plain all-white tile -> used both for the
//          START tile and the FINISH tile in Section B.
// ============================================================================

#include <Arduino.h>
#include "config.h"

class LineSensorArray {
public:
    void begin();

    // Reads and caches all 8 channels. Call once per control loop iteration
    // before using any of the getters below.
    void readAll();

    // Weighted line position: -100 (line fully to the left) .. +100 (fully
    // right), 0 = centered. Returns 0 if no sensor currently sees black
    // (line lost) -- callers should track "last known side" separately if
    // they need lost-line recovery behavior.
    int16_t getLineError() const;

    bool isBlack(uint8_t index) const;
    bool allBlack() const;
    bool allWhite() const;

    // True only when just the center sensor(s) see black while the rest of
    // the array sees white -- the signature of the small 3cm black square
    // inside the 9cm white approach-marker square. TODO: tune against the
    // real marker at approach speed; robot must be roughly centered on the
    // tile for this heuristic to trigger reliably.
    bool detectApproachMarker() const;

    // True when the sensors under the robot's footprint are predominantly
    // white -- used to recognize the START tile and the FINISH tile, both
    // of which are described as "full-white" in the brief.
    bool isOnWhiteTile() const;

private:
    int _raw[LINE_SENSOR_COUNT];
    bool _black[LINE_SENSOR_COUNT];
};
