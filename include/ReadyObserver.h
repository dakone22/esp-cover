#ifndef READYOBSERVER_H
#define READYOBSERVER_H

#include <functional>
#include <utility>

class IReadyObserver {
public:
    virtual void setCoverReady() = 0;
    virtual void setMqttConnected(bool is_connected) = 0;

    virtual ~IReadyObserver() = default;
};


class ReadyObserver : public IReadyObserver {
private:
    bool _cover_was_ready = false;
    bool _mqtt_was_connected = false;
    std::function<void()> _onAllReadyFunc;

public:
    explicit ReadyObserver(std::function<void()> onAllReadyFunc) : _onAllReadyFunc(std::move(onAllReadyFunc)) {}

    void setCoverReady() override {
        if (_mqtt_was_connected and not _cover_was_ready) {
            _onAllReadyFunc();
        }
        _cover_was_ready = true;
    }

    void setMqttConnected(bool is_connected) override {
        if (_cover_was_ready and not _mqtt_was_connected and is_connected) {
            _onAllReadyFunc();
        }
        _mqtt_was_connected = is_connected;
    }
};

#endif //READYOBSERVER_H
