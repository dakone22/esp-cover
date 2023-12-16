#include <PubSubClient.h>

#include "cover.h"
#include "config/mqtt_topics.h"

#include "MqttCover.h"

MqttCover::MqttCover(int closedPosition, int openedPosition, std::function<void(CoverState, int)> publish_func) : Cover(closedPosition, openedPosition), _publish_func(std::move(publish_func)) { }

void MqttCover::setCurrentPosition(int value) {
    Cover::setCurrentPosition(value);
    _publish_func(getState(), getCurrentPosition());
}

std::function<void(CoverState, int)> get_publish_state_and_position_func(PubSubClient& mqtt_client) {
    return [&] (CoverState state, int position) {
        if (not mqtt_client.connected()) return;

        static int lastPosition;
        if (position != lastPosition) {
            mqtt_client.publish(mqtt_topic_position, String(position).c_str());
            lastPosition = position;
        }

        static CoverState lastState;
        if (state != lastState) {
            lastState = state;
            switch (state) {
                case CoverState::Unknown:
                case CoverState::Opened:
                    mqtt_client.publish(mqtt_topic_state, mqtt_topic_state_payload_opened);
                    break;
                case CoverState::Closed:
                    mqtt_client.publish(mqtt_topic_state, mqtt_topic_state_payload_closed);
                    break;
                case CoverState::Opening:
                    mqtt_client.publish(mqtt_topic_state, mqtt_topic_state_payload_opening);
                    break;
                case CoverState::Closing:
                    mqtt_client.publish(mqtt_topic_state, mqtt_topic_state_payload_closing);
                    break;

                default:
                    break;
            }
        }
    };
}