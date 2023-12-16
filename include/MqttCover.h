#ifndef MQTTCOVER_H
#define MQTTCOVER_H

class MqttCover : public Cover {
private:
    PubSubClient _mqtt_client;

    std::function<void(CoverState, int)> _publish_func;

public:
    MqttCover(int closedPosition, int openedPosition, std::function<void(CoverState, int)> publish_func);

    void setCurrentPosition(int value) override;
};

std::function<void(CoverState, int)> get_publish_state_and_position_func(PubSubClient& mqtt_client);

#endif //MQTTCOVER_H
