#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "cover.h"
#include "sensors.h"
#include "motor.h"
#include "MqttConnectionWrapper.h"
#include "MqttCallbackWrapper.h"
#include "MqttCover.h"
#include "CoverController.h"
#include "CoverPreparer.h"
#include "CoverMovementProcessor.h"
#include "CoverTimingsManager.h"
#include "ReadyObserver.h"
#include "service.h"
#include "config.h"

const char *client_id = ("esp8266_" + String(ESP.getChipId())).c_str();
const unsigned long BAUDRATE = 921600;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

std::shared_ptr<ICoverSensors> sensors = nullptr;
std::shared_ptr<ISpeedableMotorController> motor = nullptr;
std::shared_ptr<IPositionCover> cover = nullptr;
std::shared_ptr<IMqttConnectionWrapper> mqttConnectionWrapper = nullptr;
std::shared_ptr<IMqttCallbackWrapper> mqttCallbackWrapper = nullptr;
std::shared_ptr<IReadyObserver> readyObserver = nullptr;
std::shared_ptr<ICoverTimingsManager> coverTimingsManager = nullptr;
std::shared_ptr<ICoverPreparer> coverPreparer = nullptr;
std::shared_ptr<ICoverMovementProcessor> coverMovementProcessor = nullptr;

#define SPEED 255

void setup_cover() {
    // region sensors
    sensors = std::make_shared<CoverSensors>(sensor_closed_pin,
                                             sensor_opened_pin,
                                             LOW);
    Serial.printf("CLOSED: %d; OPENED: %d\n", sensors->isClosed(), sensors->isOpened());
    // endregion

    // region motors
    motor = std::make_shared<MotorDriverController>(motor_controller_in_1_pin,
                                                    motor_controller_in_2_pin,
                                                    motor_controller_pwm_speed_pin, 0, 255);
    motor->setSpeed(SPEED);
    // endregion

    cover = std::make_shared<MqttCover>(position_closed, position_open,
                                        get_publish_state_and_position_func(mqttClient));

    readyObserver = std::make_shared<ReadyObserver>([]() {
        Serial.printf("Sending available! pos:%i; connected:%i\n", cover->getCurrentPosition(), mqttClient.connected());
        mqttClient.publish(mqtt_topic_availability, mqtt_topic_availability_payload_available);
        cover->setCurrentPosition(cover->getCurrentPosition());
    });

    coverTimingsManager = std::make_shared<CoverTimingsManager>();

    coverPreparer = std::make_shared<CoverPreparer>(sensors, motor, coverTimingsManager->loadTimeToOpen().value_or(-1),
                                                    coverTimingsManager->loadTimeToClose().value_or(-1));
}

void setup_mqtt() {

//    mqttClient.setKeepAlive(2);

    mqttConnectionWrapper = std::make_shared<MqttConnectionWrapper>(mqttClient);
    mqttConnectionWrapper->setServer({ mqtt_host, mqtt_port });
    mqttConnectionWrapper->setAuth({ client_id, mqtt_login, mqtt_pass });
    mqttConnectionWrapper->setLastWill({{ mqtt_topic_availability, MQTT::QoS::AtLeastOnce },
                                        MQTT::RetainMessage::Yes, mqtt_topic_availability_payload_not_available });
    mqttConnectionWrapper->setTopics({
                                             { mqtt_topic_set,          mqtt_topic_set_qos },
                                             { mqtt_topic_set_position, mqtt_topic_set_position_qos },
                                     });
    mqttConnectionWrapper->setOnConnectionFunc([](bool success) {
        if (success) {
            Serial.printf("mqtt connected!%i\n", mqttClient.connected());
            readyObserver->setMqttConnected(true);
        } else {
            Serial.print("failed, error code: ");
            Serial.print(mqttClient.state());
            Serial.println("!");
        }
    });
    mqttConnectionWrapper->setOnDisconnectionFunc([]() {
        Serial.print("Mqtt Disconnected!");
        readyObserver->setMqttConnected(false);
    });

    TopicHandlers topic_handlers = {
            { mqtt_topic_set_position, mqtt_topic_set_position_handler },
            { mqtt_topic_set,          mqtt_topic_set_handler },
    };
    mqttCallbackWrapper = std::make_shared<MqttCallbackWrapper>(
            mqttClient,
            std::make_shared<CoverController>(cover),
            topic_handlers);
}

void setup() {
    // region Настройка COM-порта
    Serial.begin(BAUDRATE);
    Serial.println();
    Serial.println("MQTT COVER FOR ESP8266");
    // endregion

    setup_cover();
    setup_mqtt();

    delay(5000);
}

void loop() {
    // Проверяем подключение к WiFi
    if (wifiConnectionProcess()) {
        // Подключение к WiFi установлено
        if (mqttConnectionWrapper->isConnected()) {
            // Подключение к MQTT установлено
            mqttClient.loop();
        } else {
            mqttConnectionWrapper->connect();
        }
    }

    static bool isCoverReady = false;
    if (not isCoverReady) {
        coverPreparer->prepare();
        if (coverPreparer->isPrepared()) {
            Serial.println("Cover is ready!");
            isCoverReady = true;

            auto result = coverPreparer->getResult();
            coverTimingsManager->saveTimeToOpen({ SPEED, result.timeToOpen });
            coverTimingsManager->saveTimeToClose({ SPEED, result.timeToClose });

            if (result.lastState == CoverState::Closed)
                cover->setCurrentPosition(cover->getClosedPosition());
            else if (result.lastState == CoverState::Opened)
                cover->setCurrentPosition(cover->getOpenedPosition());
            else {
                Serial.println("Unknown start state after prepare!");
            }

            if (cover->getTargetPosition() == -1) cover->setTargetPosition(cover->getCurrentPosition());

            coverMovementProcessor = std::make_shared<PositionCoverMovementProcessor>(
                    cover, motor, sensors,
                    result.timeToOpen, result.timeToClose);

            readyObserver->setCoverReady();
        }
    } else {
        coverMovementProcessor->process();
    }
}

