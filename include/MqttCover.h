#ifndef MQTTCOVER_H
#define MQTTCOVER_H

/**
 * @brief Конкретная реализация класса Cover с интеграцией MQTT: при установке текущего положения вызывает данную ей функцию
 */
class MqttCover : public Cover {
private:
    PubSubClient _mqtt_client; /**< Экземпляр клиента MQTT для связи. */
    std::function<void(CoverState, int)> _publish_func; /**< Функция обратного вызова для публикации сообщений MQTT. */

public:
    /**
     * @brief Конструктор для класса MqttCover.
     * @param closedPosition Закрытое положение шторы.
     * @param openedPosition Открытое положение шторы.
     * @param publish_func Функция обратного вызова при установки текущей позиции.
     */
    MqttCover(int closedPosition, int openedPosition, std::function<void(CoverState, int)> publish_func);

    /**
     * @brief Устанавливает текущее положение шторы.
     * @param value Текущее значение позиции.
     */
    void setCurrentPosition(int value) override;
};

#endif //MQTTCOVER_H
