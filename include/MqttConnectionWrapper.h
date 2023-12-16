#ifndef MQTTCONNECTIONWRAPPER_H
#define MQTTCONNECTIONWRAPPER_H

#include <PubSubClient.h>

#include <utility>

#include "mqtt.h"


struct TopicDefinition {
    const char *topic;
    MQTT::QoS qos;
};


struct LastWill {
    TopicDefinition topicDefinition;
    MQTT::RetainMessage retainMessage;
    const char *payload;
};

struct MqttServer {
    const char *host;
    uint16_t port;
};

struct MqttServerAuth {
    const char *client_id;
    const char *login;
    const char *password;
};

class IMqttConnectionWrapper {
private:
    virtual bool tryConnect() = 0;
    virtual bool tryConnect(LastWill lastWill) = 0;
    virtual LastWill getLastWill() = 0;
    virtual bool isLastWillSet() = 0;

    virtual void onConnectionSuccess() = 0;
    virtual void onConnectionFailed() = 0;

public:
    virtual void setServer(MqttServer mqtt_server) = 0;
    virtual void setAuth(MqttServerAuth mqtt_server_auth) = 0;
    virtual void setLastWill(LastWill last_will) = 0;
    virtual void setTopics(std::vector<TopicDefinition> topics) = 0;

    virtual void setOnConnectionFunc(std::function<void(bool)> onConnectionFunc) = 0;
    virtual void setOnDisconnectionFunc(std::function<void()> onDisconnectionFunc) = 0;

    virtual bool isConnected() = 0;
    virtual void connect() { (isLastWillSet() ? tryConnect(getLastWill()) : tryConnect()) ? onConnectionSuccess() : onConnectionFailed(); };

    virtual ~IMqttConnectionWrapper() = default;
};

class MqttConnectionWrapper : public IMqttConnectionWrapper {
private:
    PubSubClient & _mqtt_client;
    std::function<void(bool)> _onConnectionFunc;
    std::function<void()> _onDisconnectionFunc;

    MqttServerAuth _mqtt_server_auth{};

    bool is_last_will_set = false;
    LastWill _last_will{};

    std::vector<TopicDefinition> _topics;

    bool _was_connected = false;

    bool tryConnect() override;
    bool tryConnect(LastWill lastWill) override;


    inline LastWill getLastWill() override { return _last_will; }
    inline bool isLastWillSet() override { return is_last_will_set; }

    void onConnectionSuccess() override;
    void onConnectionFailed() override;

public:
    explicit MqttConnectionWrapper(PubSubClient & mqtt_client);

    inline void setServer(MqttServer mqtt_server) override { _mqtt_client.setServer(mqtt_server.host, mqtt_server.port); }
    inline void setAuth(MqttServerAuth mqtt_server_auth) override { _mqtt_server_auth = mqtt_server_auth; }
    inline void setTopics(std::vector<TopicDefinition> topics) override { _topics = std::move(topics); }

    inline void setLastWill(LastWill last_will) override { _last_will = last_will; is_last_will_set = true; }

    inline void setOnConnectionFunc(std::function<void(bool)> onConnectionFunc) override { _onConnectionFunc=std::move(onConnectionFunc); }
    inline void setOnDisconnectionFunc(std::function<void()> onDisconnectionFunc) override { _onDisconnectionFunc=std::move(onDisconnectionFunc); }


    bool isConnected() override;

};

#endif //MQTTCONNECTIONWRAPPER_H
