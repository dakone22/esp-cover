#ifndef COVERPREPARER_H
#define COVERPREPARER_H


#include <Arduino.h>

#include "cover.h"
#include "sensors.h"
#include "motor.h"

struct CoverPrepareResult {
    unsigned long timeToOpen, timeToClose;
    CoverState lastState;
};

/// "Готовит" шторы: рассчитывает время открытия и закрытия, и устанавливает в определённое положение.
class ICoverPreparer {
public:
    virtual void prepare() = 0;
    virtual bool isPrepared() = 0;
    virtual CoverPrepareResult getResult() const = 0;

    virtual ~ICoverPreparer() = default;
};

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

    void setMsToWait(long ms) {
        msWaitedTime = millis() + ms;
        isWaiting = true;
    }

public:
    CoverPreparer(const std::shared_ptr<ICoverSensors> & coverSensors,
                  const std::shared_ptr<IMotorController> & motorController,
                  unsigned long timeToOpen = -1, unsigned long timeToClose = -1);

    CoverPrepareResult getResult() const override;

    inline bool isPrepared() override { return state == Ready; }

    void prepare() override;
};

#endif //COVERPREPARER_H
