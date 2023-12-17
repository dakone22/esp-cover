#ifndef SERVICE_H
#define SERVICE_H

#include <EEPROM.h>
#include "config/memory.h"

/**
 * @brief Попытки установить соединение с точкой доступа Wi-Fi.
 * @param wifi_config Настройки конфигурации Wi-Fi.
 * @return True, если соединение успешно, false в противном случае.
 */
bool wifiConnectionProcess(const Settings::WiFi & wifi_config);

/**
 * @brief Загрузка настроек Wi-Fi и MQTT из EEPROM.
 * @return Загруженные настройки.
 */
Settings loadSettings();

#endif //SERVICE_H
