#ifndef CONFIG_MQTT_TOPICS_H
#define CONFIG_MQTT_TOPICS_H


#include "mqtt.h"

#define TOPIC(topic)     topic

// MQTT топики
/// from broker
//// set
const char* const mqtt_topic_set                                = TOPIC("set");
const char* const mqtt_topic_set_payload_open                   = "OPEN";
const char* const mqtt_topic_set_payload_stop                   = "STOP";
const char* const mqtt_topic_set_payload_close                  = "CLOSE";
const auto        mqtt_topic_set_qos                            = MQTT::QoS::AtLeastOnce;
//// set_position
const char* const mqtt_topic_set_position                       = TOPIC("set_position");
const auto        mqtt_topic_set_position_qos                   = MQTT::QoS::AtLeastOnce;

/// to broker
//// state
const char* const mqtt_topic_state                              = TOPIC("state");
const char* const mqtt_topic_state_payload_opened               = "opened";
const char* const mqtt_topic_state_payload_opening              = "opening";
const char* const mqtt_topic_state_payload_closed               = "closed";
const char* const mqtt_topic_state_payload_closing              = "closing";
//// position
const char* const mqtt_topic_position                           = TOPIC("position");
//// availability
const char* const mqtt_topic_availability                       = TOPIC("availability");
const char* const mqtt_topic_availability_payload_available     = "online";
const char* const mqtt_topic_availability_payload_not_available = "offline";

#endif