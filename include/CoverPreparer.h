#ifndef COVERPREPARER_H
#define COVERPREPARER_H


#include <Arduino.h>

#include "cover.h"
#include "sensors.h"
#include "motor.h"

/**
 * @brief Структура, представляющая результат подготовки шторы.
 */
struct CoverPrepareResult {
    unsigned long timeToOpen; /**< Время, необходимое для полного открытия шторы. */
    unsigned long timeToClose; /**< Время, необходимое для полного закрытия шторы. */
    CoverState lastState; /**< Последнее зарегистрированное состояние шторы. */
};

/**
 * @brief Интерфейс для подготовки штор: расчет времени открытия/закрытия и установка определенной позиции.
 */
class ICoverPreparer {
public:
    /**
     * @brief Подготовка обложки.
     */
    virtual void prepare() = 0;

    /**
     * @brief Проверка, подготовлена ли обложка.
     * @return True, если обложка подготовлена, false - в противном случае.
     */
    virtual bool isPrepared() = 0;

    /**
     * @brief Получает результат подготовки обложки.
     * @return CoverPrepareResult, содержащий результаты подготовки.
     */
    virtual CoverPrepareResult getResult() const = 0;

    virtual ~ICoverPreparer() = default;
};

/**
 * @brief Конкретная реализация интерфейса ICoverPreparer.
 */
class CoverPreparer : public ICoverPreparer {
    static const long MS_TO_WAIT_INERTIA = 1000;

    std::shared_ptr<ICoverSensors> _coverSensors;
    std::shared_ptr<IMotorController> _motorController;

    unsigned long msToOpen = -1;
    unsigned long msToClose = -1;

    CoverState lastCoverState = CoverState::Unknown;

    enum State {
        Unknown, UnknownMoveDown,
        StartRecordingMsToOpen, RecordedMsToOpen,
        StartRecordingMsToClose, RecordedMsToClose,
        Ready
    } state = Unknown;

    bool isWaiting = false;
    unsigned long msWaitedTime;

    /**
     * @brief Устанавливает время ожидания указанной продолжительности.
     * @param ms Продолжительность ожидания в миллисекундах.
     */
    void setMsToWait(long ms) {
        msWaitedTime = millis() + ms;
        isWaiting = true;
    }

public:
    /**
     * @brief Конструктор для класса CoverPreparer.
     * @param coverSensors Указатель на экземпляр датчиков шторы.
     * @param motorController Указатель на экземпляр контроллера моторов.
     * @param timeToOpen Загруженное время, необходимое для полного открытия крышки.
     * @param timeToClose Загруженное время, необходимое для полного закрытия крышки.
     */
    CoverPreparer(const std::shared_ptr<ICoverSensors> & coverSensors,
                  const std::shared_ptr<IMotorController> & motorController,
                  unsigned long timeToOpen = -1, unsigned long timeToClose = -1);

    CoverPrepareResult getResult() const override;

    inline bool isPrepared() override { return state == Ready; }

    void prepare() override;
};

#endif //COVERPREPARER_H
