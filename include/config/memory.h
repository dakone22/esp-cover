#ifndef CONFIG_MEMORY_H
#define CONFIG_MEMORY_H

#include "CoverTimingsManager.h"

constexpr auto MAX_STRING_SIZE = 32;

struct Settings {
    bool valid = false;

    struct WiFi {
        char ssid[MAX_STRING_SIZE];
        char pass[MAX_STRING_SIZE];
    } wifi{};

    struct MQTT {
        char host[MAX_STRING_SIZE];
        unsigned short port;
        char login[MAX_STRING_SIZE];
        char pass[MAX_STRING_SIZE];
        char topic_prefix[MAX_STRING_SIZE];
    } mqtt{};
};

namespace Offsets {
    constexpr auto CoverTimingsOpen = 0;
    constexpr auto CoverTimingsClose = sizeof(CoverTimingsEntry) * 1;
    constexpr auto Settings = sizeof(CoverTimingsEntry) * 2;
}

constexpr auto EEPROM_SIZE = Offsets::Settings + sizeof(Settings);

#endif //CONFIG_MEMORY_H
