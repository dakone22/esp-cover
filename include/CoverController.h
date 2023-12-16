#ifndef COVERCONTROLLER_H
#define COVERCONTROLLER_H

#include "cover.h"

/// Интерфейс для обработчика MQTT-запросов
class ICoverController {
public:
    enum State {
        OPEN, CLOSE, STOP
    };

    virtual void onSetPosition(int position) = 0;
    virtual void onSet(State) = 0;

    virtual ~ICoverController() = default;
};

class CoverController : public ICoverController {
private:
    std::shared_ptr<IPositionCover> _cover;
public:
    explicit CoverController(std::shared_ptr<IPositionCover> cover);

    inline void onSetPosition(int position) override { _cover->setTargetPosition(position); };
    inline void onSet(State state) override {
        if (state == STOP) {
            _cover->stop();
        } else if (state == OPEN) {
            _cover->open();
        } else if (state == CLOSE) {
            _cover->close();
        }
    };
};

#endif //COVERCONTROLLER_H
