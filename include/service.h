#ifndef SERVICE_H
#define SERVICE_H

#include <EEPROM.h>
#include "config/memory.h"

// Подключение к WiFi точке доступа
bool wifiConnectionProcess(const Settings::WiFi &);

Settings loadSettings();

#endif //SERVICE_H
