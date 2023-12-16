#include <ESP8266WiFi.h>

#include "config/wifi.h"
#include "service.h"

bool wifiConnectionProcess() {
    static enum State {
        Start, Connecting, Connected, LostConnection
    } state = Start;
    static bool isWaiting = false;
    static unsigned long nextTime;

    // Если подключение активно, то просто выходим и возвращаем true
    if (WiFi.status() == WL_CONNECTED) return true;

    while (true) {
        if (isWaiting) {
            if (millis() > nextTime)
                isWaiting = false;
            else
                return false;
        }

        switch (state) {

            case LostConnection:
                Serial.println("Lost connection, attempting again...");
                state = Start;

            case Start:
                // ... иначе пробуем подключиться к сети
                Serial.print("Connecting to WiFi AP ");
                Serial.println(wifi_ssid);

                // Настраиваем WiFi
                WiFi.mode(WIFI_STA);
                WiFi.begin(wifi_ssid, wifi_pass);

                state = Connecting;

            case Connecting:
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("Connecting...");

                    isWaiting = true;
                    nextTime = millis() + 500;

                    return false;
                }

                // Подключение успешно установлено
                Serial.println(" ок");
                Serial.print("WiFi connected, obtained IP address: ");
                Serial.println(WiFi.localIP());

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
