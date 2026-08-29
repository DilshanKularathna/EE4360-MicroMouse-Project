#pragma once
// ============================================================================
// MazePersistentStore
//
// Stores the two fully explored wall maps and their goals in the Mega's
// EEPROM.  This makes a reset/power cycle between trial attempts safe: the
// next boot can execute the learned shortest route without reprogramming or
// receiving any external maze information.
// ============================================================================

#include <Arduino.h>
#include "MazeMapper.h"

class MazePersistentStore {
public:
    // Restores a complete, checksum-verified map pair.  Returns false for a
    // blank EEPROM, a partially written record, or an incompatible layout.
    bool load(MazeMapper &mazeA, uint8_t &goalAX, uint8_t &goalAY,
              MazeMapper &mazeB, uint8_t &goalBX, uint8_t &goalBY) const;

    // Called once both section maps and their goals are complete.  This is a
    // safe checkpoint even if the final physical speed pass later fails.
    void save(const MazeMapper &mazeA, uint8_t goalAX, uint8_t goalAY,
              const MazeMapper &mazeB, uint8_t goalBX, uint8_t goalBY) const;

    // Useful before a new, different maze is introduced.  This is not used
    // during trials; it deliberately requires a firmware-side action.
    void clear() const;

private:
    static const uint16_t MAGIC = 0x4D5A; // "MZ"
    static const uint8_t VERSION = 1;
    static const uint8_t SECTION_A_CELLS = 16;
    static const uint8_t SECTION_B_CELLS = 81;

    struct Record {
        uint16_t magic;
        uint8_t version;
        uint8_t widthA;
        uint8_t heightA;
        uint8_t widthB;
        uint8_t heightB;
        uint8_t goalAX;
        uint8_t goalAY;
        uint8_t goalBX;
        uint8_t goalBY;
        uint8_t wallsA[SECTION_A_CELLS];
        uint8_t wallsB[SECTION_B_CELLS];
        uint16_t checksum;
    };

    static uint16_t checksum(const Record &record);
};
