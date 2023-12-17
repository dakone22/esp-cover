#ifndef READYOBSERVER_H
#define READYOBSERVER_H

#include <functional>
#include <utility>

/**
 * @brief Интерфейс для наблюдателя готовности.
 */
class IReadyObserver {
public:
    /**
     * @brief Уведомляет о готовности крышки.
     */
    virtual void setCoverReady() = 0;

    /**
     * @brief Сообщает о состоянии MQTT-соединения.
     * @param is_connected True, если подключен, false в противном случае.
     */
    virtual void setMqttConnected(bool is_connected) = 0;

    virtual ~IReadyObserver() = default;
};

/**
 * @brief Конкретная реализация интерфейса IReadyObserver.
 */
class ReadyObserver : public IReadyObserver {
private:
    bool _cover_was_ready = false; /**< Флаг, указывающий, готова ли штора. */
    bool _mqtt_was_connected = false; /**< Флаг, указывающий на состояние MQTT-соединения. */
    std::function<void()> _onAllReadyFunc; /**< Функция обратного вызова, когда штора и MQTT готовы. */

public:
    /**
     * @brief Конструктор для класса ReadyObserver.
     * @param onAllReadyFunc Функция обратного вызова, когда и штора, и MQTT готовы.
     */
    explicit ReadyObserver(std::function<void()> onAllReadyFunc);

    void setCoverReady() override;

    void setMqttConnected(bool is_connected) override;
};

#endif //READYOBSERVER_H
