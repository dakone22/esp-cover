#ifndef MQTTCALLBACKWRAPPER_H
#define MQTTCALLBACKWRAPPER_H

#include "CoverController.h"

class IMqttCallbackWrapper {
public:
    virtual void onMessage(char *topic, byte *payload, unsigned int length) = 0;

    virtual ~IMqttCallbackWrapper() = default;
};

typedef std::function<void(const std::shared_ptr<ICoverController> &, const String &)> TopicHandler;
typedef std::unordered_map<const char *, TopicHandler> TopicHandlers;

class MqttCallbackWrapper : public IMqttCallbackWrapper {
private:
    std::shared_ptr<ICoverController> _cover_controller;
    PubSubClient & _mqtt_client;
    std::unordered_map<const char *, TopicHandler> _topic_handlers;

public:
    MqttCallbackWrapper(PubSubClient & mqtt_client,
                        std::shared_ptr<ICoverController> cover_controller,
                        TopicHandlers topic_handlers
                );

    // Функция обратного вызова при поступлении входящего сообщения от брокера
    void onMessage(char *topic, byte *payload, unsigned int length) override;
};

void mqtt_topic_set_handler(const std::shared_ptr<ICoverController> & cover_controller, const String & payload);

void mqtt_topic_set_position_handler(const std::shared_ptr<ICoverController> & cover_controller, const String & payload);

#endif //MQTTCALLBACKWRAPPER_H
