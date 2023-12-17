#ifndef MQTTCONNECTIONWRAPPER_H
#define MQTTCONNECTIONWRAPPER_H

#include <PubSubClient.h>

#include "mqtt.h"

/**
 * @brief Структура, представляющая определение темы.
 */
struct TopicDefinition {
    const char *topic; /**< Тема MQTT. */
    MQTT::QoS qos; /**< Уровень качества обслуживания (QoS) для данной темы. */
};

/**
 * @brief Структура, представляющая конфигурацию "последней воли" клиента для MQTT-соединения.
 */
struct LastWill {
    const char *topic; /**< Тема MQTT для последней воли. */
    MQTT::QoS qos; /**< Уровень качества обслуживания (QoS) для сообщения последней воли. */
    MQTT::RetainMessage retainMessage; /**< Статус сохранения для сообщения последней воли. */
    const char *payload; /**< Полезная нагрузка для сообщения последней воли. */
};

/**
 * @brief Структура, представляющая конфигурацию сервера MQTT.
 */
struct MqttServer {
    const char *host; /**< Имя хоста или IP-адрес сервера MQTT. */
    uint16_t port; /**< Номер порта сервера MQTT. */
};

/**
 * @brief Структура, представляющая конфигурацию аутентификации для подключения к серверу MQTT.
 */
struct MqttServerAuth {
    const char *client_id; /**< Идентификатор клиента для MQTT-соединения. */
    const char *login; /**< Логин (имя пользователя) для MQTT-соединения. */
    const char *password; /**< Пароль для MQTT-соединения. */
};

/**
 * @brief Интерфейс для оберток соединений MQTT.
 */
class IMqttConnectionWrapper {
private:
    virtual bool tryConnect() = 0;
    virtual bool tryConnect(LastWill lastWill) = 0;
    virtual LastWill getLastWill() = 0;
    virtual bool isLastWillSet() = 0;

    virtual void onConnectionSuccess() = 0;
    virtual void onConnectionFailed() = 0;

public:
    /**
     * @brief Устанавливает конфигурацию сервера MQTT.
     * @param mqtt_server Конфигурация сервера MQTT.
     */
    virtual void setServer(MqttServer mqtt_server) = 0;

    /**
      * @brief Устанавливает конфигурацию аутентификации сервера MQTT.
      * @param mqtt_server_auth Конфигурация аутентификации для подключения к серверу MQTT.
      */
    virtual void setAuth(MqttServerAuth mqtt_server_auth) = 0;

    /**
     * @brief Устанавливает конфигурацию "последней воли" для MQTT-соединения.
     * @param last_will Конфигурация последней воли.
     */
    virtual void setLastWill(LastWill last_will) = 0;

    /**
     * @brief Задает список тем MQTT, на которые нужно подписаться.
     * @param topics Список структур TopicDefinition, представляющих темы MQTT.
     */
    virtual void setTopics(std::vector<TopicDefinition> topics) = 0;

    /**
     * @brief Устанавливает функцию обратного вызова для статуса соединения.
     * @param onConnectionFunc Функция обратного вызова для определения состояния соединения.
     */
    virtual void setOnConnectionFunc(std::function<void(bool)> onConnectionFunc) = 0;

    /**
     * @brief Устанавливает функцию обратного вызова для отключения.
     * @param onDisconnectionFunc Функция обратного вызова для отключения.
     */
    virtual void setOnDisconnectionFunc(std::function<void()> onDisconnectionFunc) = 0;

    /**
     * @brief Проверяет, подключен ли в данный момент MQTT-клиент..
     * @return True, если подключено, false в противном случае.
     */
    virtual bool isConnected() = 0;

    virtual void connect() { (isLastWillSet() ? tryConnect(getLastWill()) : tryConnect()) ? onConnectionSuccess() : onConnectionFailed(); };

    virtual ~IMqttConnectionWrapper() = default;
};

/**
 * @brief Конкретная реализация интерфейса IMqttConnectionWrapper.
 */
class MqttConnectionWrapper : public IMqttConnectionWrapper {
private:
    PubSubClient &_mqtt_client; /**< Ссылка на экземпляр PubSubClient. */
    std::function<void(bool)> _onConnectionFunc; /**< Функция обратного вызова для определения состояния соединения. */
    std::function<void()> _onDisconnectionFunc; /**< Функция обратного вызова для отключения. */

    MqttServerAuth _mqtt_server_auth{}; /**< Настройка аутентификации сервера MQTT. */

    bool is_last_will_set = false; /**< Флаг, указывающий, установлено ли последнее завещание. */
    LastWill _last_will{}; /**< Конфигурация последней воли. */

    std::vector<TopicDefinition> _topics; /**< Список тем MQTT для подписки. */

    bool _was_connected = false; /**< Флаг, указывающий, был ли клиент подключен в предыдущем состоянии. */


    bool tryConnect() override;
    bool tryConnect(LastWill lastWill) override;


    inline LastWill getLastWill() override { return _last_will; }
    inline bool isLastWillSet() override { return is_last_will_set; }

    void onConnectionSuccess() override;
    void onConnectionFailed() override;

public:
    /**
     * @brief Конструктор для класса MqttConnectionWrapper.
     * @param mqtt_client Ссылка на экземпляр PubSubClient.
     */
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
