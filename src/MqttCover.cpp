#include <PubSubClient.h>

#include "cover.h"

#include "MqttCover.h"

MqttCover::MqttCover(int closedPosition, int openedPosition, std::function<void(CoverState, int)> publish_func) : Cover(closedPosition, openedPosition), _publish_func(std::move(publish_func)) { }

void MqttCover::setCurrentPosition(int value) {
    Cover::setCurrentPosition(value);
    _publish_func(getState(), getCurrentPosition());
}