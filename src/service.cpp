#include <ESP8266WiFi.h>

#include "service.h"

bool wifiConnectionProcess(const Settings::WiFi & wifi) {
    static enum State {
        Start, Connecting, Connected, LostConnection
    } state = Start;
    static bool isWaiting = false;
    static unsigned long nextTime;

    while (true) {
        if (isWaiting) {
            if (millis() > nextTime)
                isWaiting = false;
            else
                return false;
        }

        switch (state) {

            case LostConnection:
#ifdef DEBUG_SERIAL_OUT
                Serial.println("Lost connection, attempting again...");
#endif
                state = Start;

            case Start:
                // ... иначе пробуем подключиться к сети
#ifdef DEBUG_SERIAL_OUT
                Serial.print("Connecting to WiFi AP ");
                Serial.println(wifi.ssid);
#endif

                // Настраиваем WiFi
                WiFi.mode(WIFI_STA);
                WiFi.begin(wifi.ssid, wifi.pass);

                state = Connecting;

            case Connecting:
                if (WiFi.status() != WL_CONNECTED) {
#ifdef DEBUG_SERIAL_OUT
                    Serial.println("Connecting...");
#endif
                    isWaiting = true;
                    nextTime = millis() + 500;

                    return false;
                }

                // Подключение успешно установлено
#ifdef DEBUG_SERIAL_OUT
                Serial.println(" ok");
                Serial.print("WiFi connected, obtained IP address: ");
                Serial.println(WiFi.localIP());
#endif
                state = Connected;

            case Connected:
                if (WiFi.status() != WL_CONNECTED) {
                    state = LostConnection;
                    break;
                }
                return true;

            default:
                break;
        }
    }
}

Settings loadSettings() {
    static bool read = false;
    static Settings settings;

    if (not read) {
        EEPROM.begin(EEPROM_SIZE);

        EEPROM.get(Offsets::Settings, settings);

        EEPROM.end();
        read = true;
    }

    return settings;
}
