#include <PubSubClient.h>

#include "config/mqtt_topics.h"
#include "MqttCallbackWrapper.h"
#include "CoverController.h"

MqttCallbackWrapper::MqttCallbackWrapper(PubSubClient & mqtt_client,
                                         std::shared_ptr<ICoverController> cover_controller,
                                         TopicHandlers topic_handlers)
        : _cover_controller(std::move(cover_controller)), _mqtt_client(mqtt_client), _topic_handlers(std::move(topic_handlers)) {
    _mqtt_client.setCallback([this](char *topic, uint8_t *payload, unsigned int length) {
        this->onMessage(topic, payload, length);
    });
}

void MqttCallbackWrapper::onMessage(char *topic, byte *payload, unsigned int length) {
    // Для более корректного сравнения строк обрезаем пробелы с краев
    String _payload;
    for (unsigned int i = 0; i < length; i++) {
        _payload += String((char) payload[i]);
    };
    _payload.trim();

    // Вывод поступившего сообщения в лог, больше никакого смысла этот блок кода не несет, можно исключить
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.print(_payload.c_str());
    Serial.println();

    for (const auto& [key, _] : _topic_handlers)
        Serial.printf("key: %s\n", key);

    // Сравниваем с топиками
    for (const auto& [_topic, func] : _topic_handlers)
        if (strcmp(topic, _topic) == 0) {
            func(_cover_controller, _payload);
            return;
        }
    Serial.println("Failed to recognize incoming with_prefix!");
}

void mqtt_topic_set_handler(const std::shared_ptr<ICoverController> & cover_controller, const String & payload) {
    if (payload.equals(mqtt_topic_set_payload_open)) {
        cover_controller->onSet(ICoverController::OPEN);
    } else if (payload.equals(mqtt_topic_set_payload_close)) {
        cover_controller->onSet(ICoverController::CLOSE);
    } else if (payload.equals(mqtt_topic_set_payload_stop)) {
        cover_controller->onSet(ICoverController::STOP);
    } else {
        Serial.println("Unknown payload: " + payload);
    }
}

void mqtt_topic_set_position_handler(const std::shared_ptr<ICoverController> & cover_controller, const String & payload) {
    int pos = payload.toInt();
    cover_controller->onSetPosition(pos);
}
