#include "MazePersistentStore.h"

#include <EEPROM.h>
#include <stddef.h>

uint16_t MazePersistentStore::checksum(const Record &record) {
    // Fletcher-16 gives an inexpensive corruption check and, unlike a bare
    // magic value, rejects incomplete writes if power is removed mid-save.
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
    const size_t count = offsetof(Record, checksum);
    uint16_t sum1 = 0xFF;
    uint16_t sum2 = 0xFF;
    for (size_t i = 0; i < count; i++) {
        sum1 = static_cast<uint16_t>((sum1 + bytes[i]) % 255);
        sum2 = static_cast<uint16_t>((sum2 + sum1) % 255);
    }
    return static_cast<uint16_t>((sum2 << 8) | sum1);
}

bool MazePersistentStore::load(MazeMapper &mazeA, uint8_t &goalAX, uint8_t &goalAY,
                               MazeMapper &mazeB, uint8_t &goalBX, uint8_t &goalBY) const {
    Record record{};
    EEPROM.get(0, record);

    if (record.magic != MAGIC || record.version != VERSION ||
        record.widthA != mazeA.width() || record.heightA != mazeA.height() ||
        record.widthB != mazeB.width() || record.heightB != mazeB.height() ||
        record.goalAX >= mazeA.width() || record.goalAY >= mazeA.height() ||
        record.goalBX >= mazeB.width() || record.goalBY >= mazeB.height() ||
        record.checksum != checksum(record)) {
        return false;
    }

    mazeA.clear();
    mazeB.clear();
    for (uint8_t y = 0; y < mazeA.height(); y++) {
        for (uint8_t x = 0; x < mazeA.width(); x++) {
            mazeA.setWallMask(x, y, record.wallsA[static_cast<uint8_t>(y * mazeA.width() + x)]);
        }
    }
    for (uint8_t y = 0; y < mazeB.height(); y++) {
        for (uint8_t x = 0; x < mazeB.width(); x++) {
            mazeB.setWallMask(x, y, record.wallsB[static_cast<uint8_t>(y * mazeB.width() + x)]);
        }
    }

    goalAX = record.goalAX;
    goalAY = record.goalAY;
    goalBX = record.goalBX;
    goalBY = record.goalBY;
    return true;
}

void MazePersistentStore::save(const MazeMapper &mazeA, uint8_t goalAX, uint8_t goalAY,
                               const MazeMapper &mazeB, uint8_t goalBX, uint8_t goalBY) const {
    Record record{};
    record.magic = MAGIC;
    record.version = VERSION;
    record.widthA = mazeA.width();
    record.heightA = mazeA.height();
    record.widthB = mazeB.width();
    record.heightB = mazeB.height();
    record.goalAX = goalAX;
    record.goalAY = goalAY;
    record.goalBX = goalBX;
    record.goalBY = goalBY;

    for (uint8_t y = 0; y < mazeA.height(); y++) {
        for (uint8_t x = 0; x < mazeA.width(); x++) {
            record.wallsA[static_cast<uint8_t>(y * mazeA.width() + x)] = mazeA.getWallMask(x, y);
        }
    }
    for (uint8_t y = 0; y < mazeB.height(); y++) {
        for (uint8_t x = 0; x < mazeB.width(); x++) {
            record.wallsB[static_cast<uint8_t>(y * mazeB.width() + x)] = mazeB.getWallMask(x, y);
        }
    }
    record.checksum = checksum(record);

    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
    // Invalidate first, then commit the magic bytes last.  A reset during the
    // write therefore leaves either the old record invalid or the new one
    // checksum-verified -- never a route we merely hope is complete.
    EEPROM.update(0, 0);
    EEPROM.update(1, 0);
    for (uint16_t i = 2; i < sizeof(Record); i++) EEPROM.update(i, bytes[i]);
    EEPROM.update(0, bytes[0]);
    EEPROM.update(1, bytes[1]);
}

void MazePersistentStore::clear() const {
    EEPROM.update(0, 0);
    EEPROM.update(1, 0);
}
