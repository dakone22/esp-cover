#ifndef COVERPROCESSOR_H
#define COVERPROCESSOR_H


#include <memory>
#include <utility>
#include "cover.h"
#include "motor.h"
#include "sensors.h"


/// Обрабатывает движения шторы
class ICoverMovementProcessor {
public:
    virtual void process() = 0;

    virtual ~ICoverMovementProcessor() = default;
};


class PositionCoverMovementProcessor : public ICoverMovementProcessor {
private:
    std::shared_ptr<IPositionCover> cover;
    std::shared_ptr<IMotorController> motor;
    std::shared_ptr<ICoverSensors> sensors;
    unsigned long timeToOpen, timeToClose;

public:
    PositionCoverMovementProcessor(std::shared_ptr<IPositionCover> cover_,
                                   std::shared_ptr<IMotorController> motorController_,
                                   std::shared_ptr<ICoverSensors> coverSensors_,
                                   unsigned long timeToOpen,
                                   unsigned long timeToClose);

    void process() override;
};


#endif //COVERPROCESSOR_H
