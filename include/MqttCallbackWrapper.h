#ifndef MQTTCALLBACKWRAPPER_H
#define MQTTCALLBACKWRAPPER_H

#include "CoverController.h"

/**
 * @brief Интерфейс обертки обратных вызовов MQTT-обработчика.
 */
class IMqttCallbackWrapper {
public:
    /**
     * @brief Обработка входящих сообщений MQTT.
     * @param topic Тема MQTT входящего сообщения.
     * @param payload Полезная нагрузка входящего сообщения.
     * @param length Длина полезной нагрузки.
     */
    virtual void onMessage(char *topic, byte *payload, unsigned int length) = 0;

    virtual ~IMqttCallbackWrapper() = default;
};

/**
 * @brief Typedef для функций обработчика тем.
 */
typedef std::function<void(const std::shared_ptr<ICoverController> &, const String &)> TopicHandler;

/**
 * @brief Типовое определение для ассоциативного массива обработчиков тем.
 */
typedef std::unordered_map<const char *, TopicHandler> TopicHandlers;

/**
 * @brief Конкретная реализация интерфейса IMqttCallbackWrapper.
 */
class MqttCallbackWrapper : public IMqttCallbackWrapper {
private:
    std::shared_ptr<ICoverController> _cover_controller; /**< Указатель на экземпляр контроллера шторы. */
    PubSubClient &_mqtt_client; /**< Ссылка на экземпляр клиента MQTT. */
    TopicHandlers _topic_handlers; /**< Карта обработчиков тем. */

public:
    /**
     * @brief Конструктор для класса MqttCallbackWrapper.
     * @param mqtt_client Ссылка на экземпляр PubSubClient.
     * @param cover_controller Общий указатель на экземпляр контроллера шторы.
     * @param topic_handlers Ассоциативный массив обработчиков тем.
     */
    MqttCallbackWrapper(
            PubSubClient &mqtt_client,
            std::shared_ptr<ICoverController> cover_controller,
            TopicHandlers topic_handlers
    );

    void onMessage(char *topic, byte *payload, unsigned int length) override;
};

/**
 * @brief Обрабатывает MQTT-сообщения для установки состояния шторы.
 * @param cover_controller Указатель на экземпляр контроллера шторы.
 * @param payload Полезная нагрузка входящего сообщения.
 */
void mqtt_topic_set_handler(const std::shared_ptr<ICoverController> &cover_controller, const String &payload);

/**
 * @brief Обрабатывает MQTT-сообщения для установки положения шторы.
 * @param cover_controller Указатель на экземпляр контроллера шторы.
 * @param payload Полезная нагрузка входящего сообщения.
 */
void mqtt_topic_set_position_handler(const std::shared_ptr<ICoverController> &cover_controller, const String &payload);

#endif //MQTTCALLBACKWRAPPER_H
