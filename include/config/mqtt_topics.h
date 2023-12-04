#ifndef CONFIG_MQTT_TOPICS_H
#define CONFIG_MQTT_TOPICS_H


#include "mqtt.h"

#define topic_root        "home-assistant"
#define topic_device_name "balcony-cover-1"
#define TOPIC(topic)      topic_root "/" topic_device_name "/" topic

// MQTT топики
/// from broker
//// set
const char* mqtt_topic_set                                = TOPIC("set");
const char* mqtt_topic_set_payload_open                   = "OPEN";
const char* mqtt_topic_set_payload_stop                   = "STOP";
const char* mqtt_topic_set_payload_close                  = "CLOSE";
const auto  mqtt_topic_set_qos                            = MQTT::QoS::AtLeastOnce;
//// set_position
const char* mqtt_topic_set_position                       = TOPIC("set_position");
const auto  mqtt_topic_set_position_qos                   = MQTT::QoS::AtLeastOnce;

/// to broker
//// state
const char* mqtt_topic_state                              = TOPIC("state");
const char* mqtt_topic_state_payload_open                 = "open";
const char* mqtt_topic_state_payload_opening              = "opening";
const char* mqtt_topic_state_payload_close                = "close";
const char* mqtt_topic_state_payload_closing              = "closing";
//// position
const char* mqtt_topic_position                           = TOPIC("position");
//// availability
const char* mqtt_topic_availability                       = TOPIC("availability");
const char* mqtt_topic_availability_payload_available     = "online";
const char* mqtt_topic_availability_payload_not_available = "offline";

#endif