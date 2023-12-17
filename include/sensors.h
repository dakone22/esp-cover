#ifndef SENSORS_H
#define SENSORS_H

/**
 * @brief Интерфейс для датчиков шторыи.
 */
class ICoverSensors {
public:
    /**
     * @brief Проверяет, закрыта ли штора.
     * @return True, если закрыто, false в противном случае.
     */
    virtual bool isClosed() = 0;

    /**
     * @brief Проверяет, открыта ли штора.
     * @return True, если открыто, false в противном случае.
     */
    virtual bool isOpened() = 0;

    virtual ~ICoverSensors() = default;
};

/**
 * @brief Конкретная реализация интерфейса ICoverSensors.
 */
class CoverSensors : public ICoverSensors {
private:
    uint8_t _closed_sensor_pin; /**< Пин, подключенный к датчику закрытости. */
    uint8_t _opened_sensor_pin; /**< Пин, подключенный к датчику открытости. */
    int _active_value; /**< Активное значение, указывающее на состояние датчика. */

public:
    /**
     * @brief Конструктор для класса CoverSensors.
     * @param closed_sensor_pin Пин, подключенный к датчику закрытости.
     * @param opened_sensor_pin Пин, подключенный к датчику открытости.
     * @param active_value Активное значение, указывающее на состояние датчика.
     */
    CoverSensors(uint8_t closed_sensor_pin, uint8_t opened_sensor_pin, int active_value);

    inline bool isClosed() override { return digitalRead(_closed_sensor_pin) == _active_value; }
    inline bool isOpened() override { return digitalRead(_opened_sensor_pin) == _active_value; }
};

#endif // SENSORS_H