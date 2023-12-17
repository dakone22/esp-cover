#ifndef MOTOR_H
#define MOTOR_H

/**
 * @brief Интерфейс для основных операций управления двигателем.
 */
class IMotorController {
public:
    /**
     * @brief Вращает двигатель для открытия шторы.
     */
    virtual void rotateOpening() = 0;

    /**
     * @brief Вращает двигатель для закрытия шторы.
     */
    virtual void rotateClosing() = 0;

    /**
     * @brief Остановка двигателя.
     */
    virtual void stop() = 0;

    virtual ~IMotorController() = default;
};

/**
 * @brief Интерфейс для управления двигателем с регулировкой скорости.
 */
class ISpeedableMotorController : public IMotorController {
public:
    /**
     * @brief Устанавливает скорость вращения двигателя.
     * @param value Значение скорости, которое необходимо установить.
     */
    virtual void setSpeed(int value) = 0;

    /**
     * @brief Получает текущую скорость двигателя.
     * @return Текущее значение скорости.
     */
    virtual int getCurrentSpeed() = 0;

    /**
     * @brief Получает минимально допустимую скорость вращения двигателя.
     * @return Минимальное значение скорости.
     */
    virtual int getMinSpeed() = 0;

    /**
     * @brief Получает максимально допустимую скорость вращения двигателя.
     * @return Максимальное значение скорости.
     */
    virtual int getMaxSpeed() = 0;
};

/**
 * @brief Интерфейс контроллера мотора со скорость, реализованного через ШИМ пин.
 */
class PwmSpeedableMotorController : public ISpeedableMotorController {
private:
    const int MAX_ANALOG_VALUE = 0xFF;

    uint8_t _pwm_speed_pin; /**< Пин для управления скоростью двигателя. */

    int _current_speed{}; /**< Текущая скорость двигателя. */
    int _min_speed; /**< Минимально допустимая скорость вращения двигателя. */
    int _max_speed; /**< Максимально допустимая скорость вращения двигателя. */
public:
    PwmSpeedableMotorController(uint8_t pwm_speed_pin, int min_speed, int max_speed)
            : _pwm_speed_pin(pwm_speed_pin), _min_speed(min_speed), _max_speed(max_speed) {
        pinMode(_pwm_speed_pin, OUTPUT);
        setSpeed(_max_speed);
    }

    void setSpeed(int speed) override;
    inline int getCurrentSpeed() override { return _current_speed; };

    inline int getMinSpeed() override { return _min_speed; };
    inline int getMaxSpeed() override { return _max_speed; };
};

/**
 * @brief Конкретная реализация интерфейса ISpeedableMotorController для прямого управления через два пина.
 */
class MotorDriverController : public PwmSpeedableMotorController {
private:
    uint8_t _in1_pin; /**< Пин 1 для управления направлением вращения двигателя. */
    uint8_t _in2_pin; /**< Пин 2 для управления направлением вращения двигателя. */


    /**
     * @brief Записывает заданные значения на пины управления двигателем.
     * @param in1 Значение для записи в первый пин управления двигателем.
     * @param in2 Значение для записи во второй пин управления двигателем.
     */
    inline void write_in_pins(int in1, int in2) const {
        digitalWrite(_in1_pin, in1);
        digitalWrite(_in2_pin, in2);
    }

public:
    /**
     * @brief Конструктор для класса MotorDriverController.
     * @param in1_pin Пин 1 для управления направлением вращения двигателя.
     * @param in2_pin Пин 2 для управления направлением вращения двигателя.
     * @param pwm_speed_pin Пин для управления скоростью двигателя.
     * @param min_speed Минимально допустимая скорость вращения двигателя.
     * @param max_speed Максимально допустимая скорость вращения двигателя.
     */
    MotorDriverController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_speed_pin, int min_speed, int max_speed);

    inline void rotateOpening() override { write_in_pins(LOW, HIGH); }
    inline void rotateClosing() override { write_in_pins(HIGH, LOW); }
    inline void stop() override          { write_in_pins(LOW, LOW); }

};

class BufferedMotorDriverController : public PwmSpeedableMotorController {
private:
    uint8_t * buffer;
    size_t _in1_index;
    size_t _in2_index;

    inline void write_in_buffer(int in1, int in2) const {
        buffer[_in1_index] = in1;
        buffer[_in2_index] = in2;
    }

public:
    BufferedMotorDriverController(uint8_t * buffer, size_t in1_index, size_t in2_index, uint8_t pwm_speed_pin, int min_speed, int max_speed);

    inline void rotateOpening() override { write_in_buffer(LOW, HIGH); }
    inline void rotateClosing() override { write_in_buffer(HIGH, LOW); }
    inline void stop() override          { write_in_buffer(LOW, LOW); }
};

#endif