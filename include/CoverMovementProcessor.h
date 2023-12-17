#ifndef COVERPROCESSOR_H
#define COVERPROCESSOR_H


#include <memory>
#include <utility>
#include "cover.h"
#include "motor.h"
#include "sensors.h"


/**
 * @brief Интерфейс для обработки движений крышки.
 */
class ICoverMovementProcessor {
public:
    /**
     * @brief Обработка перемещения шторы.
     */
    virtual void process() = 0;

    virtual ~ICoverMovementProcessor() = default;
};

/**
 * @brief Конкретная реализация интерфейса ICoverMovementProcessor.
 */
class PositionCoverMovementProcessor : public ICoverMovementProcessor {
private:
    std::shared_ptr<IPositionCover> cover;
    std::shared_ptr<IMotorController> motor;
    std::shared_ptr<ICoverSensors> sensors;
    unsigned long timeToOpen;
    unsigned long timeToClose;

public:
    /**
     * @brief Конструктор для класса PositionCoverMovementProcessor.
     * @param cover Указатель на экземпляр позиционной шторы.
     * @param motorController Указатель на экземпляр контроллера мотора.
     * @param coverSensors Указатель на экземпляр датчиков шторы.
     * @param timeToOpen Время, необходимое для полного открытия крышки.
     * @param timeToClose Время, необходимое для полного закрытия крышки.
     */
    PositionCoverMovementProcessor(
            std::shared_ptr<IPositionCover> cover,
            std::shared_ptr<IMotorController> motorController,
            std::shared_ptr<ICoverSensors> coverSensors,
            unsigned long timeToOpen,
            unsigned long timeToClose
    );

    void process() override;
};


#endif //COVERPROCESSOR_H
