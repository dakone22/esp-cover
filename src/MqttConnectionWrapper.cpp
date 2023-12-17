#include "MqttConnectionWrapper.h"

MqttConnectionWrapper::MqttConnectionWrapper(PubSubClient & mqtt_client) : _mqtt_client(mqtt_client) { }

bool MqttConnectionWrapper::tryConnect() {
    return _mqtt_client.connect(
            _mqtt_server_auth.client_id,
            _mqtt_server_auth.login,
            _mqtt_server_auth.password);
}

bool MqttConnectionWrapper::tryConnect(LastWill lastWill) {
#ifdef DEBUG_SERIAL_OUT
    Serial.printf("willtopic: \"%s\"\n", lastWill.topic);
#endif
    return _mqtt_client.connect(
            _mqtt_server_auth.client_id,
            _mqtt_server_auth.login,
            _mqtt_server_auth.password,
            // last will: send `not available` when disconnected
            lastWill.topic,
            lastWill.qos,
            lastWill.retainMessage,
            lastWill.payload);
}

bool MqttConnectionWrapper::isConnected() {
    auto connected = _mqtt_client.connected();
    if (not connected and _was_connected) {
        _onDisconnectionFunc();
        _was_connected = false;
    }

    return connected;
}

void MqttConnectionWrapper::onConnectionSuccess() {
    // Подписываемся на топики
    for (const auto & [topic, qos] : _topics)
        _mqtt_client.subscribe(topic, qos);

    _was_connected = true;
#ifdef DEBUG_SERIAL_OUT
    Serial.printf("calling _onConnectionFunc(true); %i\n", _mqtt_client.connected());
#endif
    _onConnectionFunc(true);
}

void MqttConnectionWrapper::onConnectionFailed() {
    _onConnectionFunc(false);
}
