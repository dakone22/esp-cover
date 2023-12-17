#include "ReadyObserver.h"

ReadyObserver::ReadyObserver(std::function<void()> onAllReadyFunc) : _onAllReadyFunc(std::move(onAllReadyFunc)) {}

void ReadyObserver::setCoverReady() {
    if (_mqtt_was_connected and not _cover_was_ready) {
        _onAllReadyFunc();
    }
    _cover_was_ready = true;
}

void ReadyObserver::setMqttConnected(bool is_connected) {
    if (_cover_was_ready and not _mqtt_was_connected and is_connected) {
        _onAllReadyFunc();
    }
    _mqtt_was_connected = is_connected;
}
