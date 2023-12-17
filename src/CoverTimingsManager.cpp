#include <Arduino.h>
#include <EEPROM.h>

#include "CoverTimingsManager.h"
#include "config/memory.h"

bool isValid(CoverTimingsEntry entry) {
    return entry.speed > 0 and
            (0 < entry.timeToMove and entry.timeToMove < 10000);
}

CoverTimingsManager::CoverTimingsManager() {
    EEPROM.begin(EEPROM_SIZE);

    EEPROM.get(Offsets::CoverTimingsOpen, toOpen);
    EEPROM.get(Offsets::CoverTimingsClose, toClose);

    EEPROM.end();

    valid = isValid(toOpen) and isValid(toClose);
#ifdef DEBUG_SERIAL_OUT
    Serial.printf("Read from memory: open=%lu (%i), close=%lu (%i)\n", toOpen.timeToMove, toOpen.speed, toClose.timeToMove, toClose.speed);
#endif
}

inline bool isEqual(CoverTimingsEntry e1, CoverTimingsEntry e2) {
    return e1.speed == e2.speed and e1.timeToMove == e2.timeToMove;
}

inline void save(int address, CoverTimingsEntry entry) {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(address, entry);
    EEPROM.end();
}

void CoverTimingsManager::saveTimeToOpen(CoverTimingsEntry entry) {
    if (isEqual(entry, toOpen)) return;

    save(Offsets::CoverTimingsOpen, entry);
}

void CoverTimingsManager::saveTimeToClose(CoverTimingsEntry entry) {
    if (isEqual(entry, toClose)) return;

    save(Offsets::CoverTimingsClose, entry);
}

std::optional<unsigned long> CoverTimingsManager::loadTimeToOpen() {
    if (valid) {
        return toOpen.timeToMove;
    } else {
        return { };
    }
}

std::optional<unsigned long> CoverTimingsManager::loadTimeToClose() {
    if (valid) {
        return toClose.timeToMove;
    } else {
        return { };
    }
}
