#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266Webserver.h>

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

auto client_id = "esp8266_" + String(ESP.getChipId());
const unsigned long BAUDRATE = 921600;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

uint8_t sensor_buffer[8];
uint8_t motor_buffer[8];
int window_indexes[] = { 0, 1, 2, 3 };

std::shared_ptr<ICoverSensors> sensors[4];
std::shared_ptr<ISpeedableMotorController> motor[4];
std::shared_ptr<IPositionCover> covers[4];
std::shared_ptr<ICoverPreparer> coverPreparer[4];
std::shared_ptr<ICoverTimingsManager> coverTimingsManager[4];
std::shared_ptr<IReadyObserver> readyObserver[4];
std::shared_ptr<ICoverMovementProcessor> coverMovementProcessor[4];

std::shared_ptr<IMqttConnectionWrapper> mqttConnectionWrapper = nullptr;
std::shared_ptr<IMqttCallbackWrapper> mqttCallbackWrapper = nullptr;

Settings settings;

// region Settings Web Page

enum Mode { HostingWebSettings, ProcessingMqttCover } working_mode;
std::shared_ptr<ESP8266WebServer> server;

// Function to handle root URL
void handleRoot() {
    String html = "<html><body>";
    html += "<h1>Settings</h1>";
    html += "<form action='/save' method='post'>";
    html += "<h2>WiFi Settings</h2>";
    html += "SSID: <input type='text' name='ssid' value='" + String(settings.wifi.ssid) + "'><br>";
    html += "Password: <input type='password' name='pass' value='" + String(settings.wifi.pass) + "'><br>";
    html += "<h2>MQTT Settings</h2>";
    html += "Host: <input type='text' name='host' value='" + String(settings.mqtt.host) + "'><br>";
    html += "Port: <input type='number' name='port' value='" + String(settings.mqtt.port) + "'><br>";
    html += "Login: <input type='text' name='login' value='" + String(settings.mqtt.login) + "'><br>";
    html += "Password: <input type='password' name='mqtt_pass' value='" + String(settings.mqtt.pass) + "'><br>";
    html += "Topic Prefix: <input type='text' name='topic_prefix' value='" + String(settings.mqtt.topic_prefix) + "'><br>";
    html += "<input type='submit' value='Save'>";
    html += "</form></body></html>";
    server->send(200, "text/html", html);
}

// Function to handle saving settings
void handleSave() {
    settings.valid = true;  // Mark settings as valid

    // WiFi settings
    String ssid = server->arg("ssid");
    String pass = server->arg("pass");
    if (ssid.length() > MAX_STRING_SIZE || pass.length() > MAX_STRING_SIZE) {
        server->send(400, "text/plain", "Error: SSID or password too long");
        return;
    }
    strncpy(settings.wifi.ssid, ssid.c_str(), MAX_STRING_SIZE);
    strncpy(settings.wifi.pass, pass.c_str(), MAX_STRING_SIZE);

    // MQTT settings
    String host = server->arg("host");
    if (host.length() > MAX_STRING_SIZE) {
        server->send(400, "text/plain", "Error: MQTT host too long");
        return;
    }
    settings.mqtt.port = server->arg("port").toInt();
    String login = server->arg("login");
    String mqtt_pass = server->arg("mqtt_pass");
    String topic_prefix = server->arg("topic_prefix");
    if (login.length() > MAX_STRING_SIZE || mqtt_pass.length() > MAX_STRING_SIZE || topic_prefix.length() > MAX_STRING_SIZE) {
        server->send(400, "text/plain", "Error: MQTT login, password, or with_prefix prefix too long");
        return;
    }
    strncpy(settings.mqtt.host, host.c_str(), MAX_STRING_SIZE);
    strncpy(settings.mqtt.login, login.c_str(), MAX_STRING_SIZE);
    strncpy(settings.mqtt.pass, mqtt_pass.c_str(), MAX_STRING_SIZE);
    strncpy(settings.mqtt.topic_prefix, topic_prefix.c_str(), MAX_STRING_SIZE);

    // Save settings to EEPROM
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(Offsets::Settings, settings);
    EEPROM.commit();
    EEPROM.end();

    server->send(200, "text/plain", "Settings saved. Rebooting...");
    delay(1000);
    ESP.reset();
}

// endregion


class Topics {
private:
    String _prefix;
    String _topic_position[4];
    String _topic_state[4];
    String _topic_availability[4];
    String _topic_set[4];
    String _topic_set_position[4];

    inline String with_prefix(int i, const char * topic) {
        return _prefix + String(i) + "/" + topic;
    }

public:
    Topics(const char * prefix) : _prefix(prefix) {
        for (int i : window_indexes) {
            _topic_position[i] = with_prefix(i, mqtt_topic_position);
            _topic_state[i] = with_prefix(i, mqtt_topic_state);
            _topic_availability[i] = with_prefix(i, mqtt_topic_availability);
            _topic_set[i] = with_prefix(i, mqtt_topic_set);
            _topic_set_position[i] = with_prefix(i, mqtt_topic_set_position);

        }
    }

    inline const char * getTopicPosition(int i) const {
        return _topic_position[i].c_str();
    }

    inline const char * getTopicState(int i) const {
        return _topic_state[i].c_str();
    }

    inline const char * getTopicAvailability(int i) const {
        return _topic_availability[i].c_str();
    }

    inline const char * getTopicSet(int i) const {
        return _topic_set[i].c_str();
    }

    inline const char * getTopicSetPosition(int i) const {
        return _topic_set_position[i].c_str();
    }
};

std::shared_ptr<Topics> _topics;

#define SPEED 255

void setup_cover() {
    for (auto i : window_indexes) {
        sensors[i] = std::make_shared<BufferedCoverSensors>(sensor_buffer, (2 * i), (2 * i) + 1, LOW);
        Serial.printf("%i: CLOSED: %d; OPENED: %d\n", i, sensors[i]->isClosed(), sensors[i]->isOpened());

        motor[i] = std::make_shared<BufferedMotorDriverController>(motor_buffer, (2 * i), (2 * i) + 1,
                                                                   motor_controller_pwm_speed_pin[i], 0, 255);
        motor[i]->setSpeed(SPEED);

        covers[i] = std::make_shared<MqttCover>(position_closed, position_open, [i](CoverState state, int position) {
            if (not mqttClient.connected()) return;

            static int lastPosition;
            if (position != lastPosition) {
                mqttClient.publish(_topics->getTopicPosition(i), String(position).c_str());
                lastPosition = position;
            }

            static CoverState lastState;
            if (state != lastState) {
                lastState = state;
                switch (state) {
                    case CoverState::Unknown:
                    case CoverState::Opened:
                        mqttClient.publish(_topics->getTopicState(i), mqtt_topic_state_payload_opened);
                        break;
                    case CoverState::Closed:
                        mqttClient.publish(_topics->getTopicState(i), mqtt_topic_state_payload_closed);
                        break;
                    case CoverState::Opening:
                        mqttClient.publish(_topics->getTopicState(i), mqtt_topic_state_payload_opening);
                        break;
                    case CoverState::Closing:
                        mqttClient.publish(_topics->getTopicState(i), mqtt_topic_state_payload_closing);
                        break;

                    default:
                        break;
                }
            }
        });

        readyObserver[i] = std::make_shared<ReadyObserver>([i]() {
            Serial.printf("%i: Sending available at \"%s\"! pos:%i; connected:%i\n", i, _topics->getTopicAvailability(i), covers[i]->getCurrentPosition(), mqttClient.connected());
            mqttClient.publish(_topics->getTopicAvailability(i), mqtt_topic_availability_payload_available);
            covers[i]->setCurrentPosition(covers[i]->getCurrentPosition());
        });

        coverTimingsManager[i] = std::make_shared<CoverTimingsManager>();

        coverPreparer[i] = std::make_shared<CoverPreparer>(sensors[i], motor[i],
                                                        coverTimingsManager[i]->loadTimeToOpen().value_or(-1),
                                                        coverTimingsManager[i]->loadTimeToClose().value_or(-1));
    }
}

void setup_mqtt() {
    mqttConnectionWrapper = std::make_shared<MqttConnectionWrapper>(mqttClient);
    mqttConnectionWrapper->setServer({ settings.mqtt.host, settings.mqtt.port });
    mqttConnectionWrapper->setAuth({ client_id.c_str(), settings.mqtt.login, settings.mqtt.pass });
    auto _mqtt_topic_availability = String(settings.mqtt.topic_prefix) + String(1) + "/" + (mqtt_topic_availability);
    mqttConnectionWrapper->setLastWill({_topics->getTopicAvailability(1), MQTT::QoS::AtLeastOnce,
                                        MQTT::RetainMessage::Yes, mqtt_topic_availability_payload_not_available });
    mqttConnectionWrapper->setTopics({
                                             { _topics->getTopicSetPosition(1), mqtt_topic_set_position_qos },
                                             { _topics->getTopicSet(1),          mqtt_topic_set_qos },
                                     });
    mqttConnectionWrapper->setOnConnectionFunc([](bool success) {
        if (success) {
            Serial.printf("mqtt connected!%i\n", mqttClient.connected());
            for (auto i : window_indexes) readyObserver[i]->setMqttConnected(true);
        } else {
            Serial.print("failed, error code: ");
            Serial.print(mqttClient.state());
            Serial.println("!");
        }
    });
    mqttConnectionWrapper->setOnDisconnectionFunc([]() {
        Serial.print("Mqtt Disconnected!");
        for (auto i : window_indexes) readyObserver[i]->setMqttConnected(false);
    });

    for (auto i : window_indexes) {
        TopicHandlers topic_handlers = {
                { _topics->getTopicSetPosition(i), mqtt_topic_set_position_handler },
                { _topics->getTopicSet(i),         mqtt_topic_set_handler },
        };
        mqttCallbackWrapper = std::make_shared<MqttCallbackWrapper>(
                mqttClient,
                std::make_shared<CoverController>(covers[i]),
                topic_handlers);
    }
}

void setup() {
    // region Настройка COM-порта
    Serial.begin(BAUDRATE);
    Serial.println();
    Serial.println("MQTT COVER FOR ESP8266");
    // endregion

    delay(5000);

    settings = loadSettings();
    if (settings.valid) {
        working_mode = Mode::ProcessingMqttCover;

        _topics = std::make_shared<Topics>(settings.mqtt.topic_prefix);
        setup_cover();
        setup_mqtt();

        pinMode(settings_reset_pin, INPUT_PULLUP);
    } else {
        server = std::make_shared<ESP8266WebServer>();
        Serial.printf("Starting wifi \"%s\" is %i\n", client_id.c_str(), WiFi.softAP(client_id.c_str(), nullptr));
        server->on("/", HTTP_GET, handleRoot);
        server->on("/save", HTTP_POST, handleSave);
        server->begin();

        Serial.print("Starting server at ");
        Serial.print(WiFi.softAPIP());
        Serial.print(":");
        Serial.println(server->getServer().port());

        working_mode = Mode::HostingWebSettings;
    }
}

void write_motor_control(const uint8_t * buffer) {
    static bool is_pin_inited = false;

    if (not is_pin_inited) {
        pinMode(motor_reg_data_pin, OUTPUT);
        pinMode(motor_reg_shift_pin, OUTPUT);
        pinMode(motor_reg_latch_pin, OUTPUT);
        digitalWrite(motor_reg_latch_pin, LOW);
    }

    digitalWrite(motor_reg_latch_pin, LOW);

    for (auto i = 0; i < 8; ++i) {
        digitalWrite(motor_reg_data_pin, buffer[i]);

        digitalWrite(motor_reg_shift_pin, HIGH);
        digitalWrite(motor_reg_shift_pin, LOW);
    }

    digitalWrite(motor_reg_latch_pin, HIGH);

}

void read_sensors(uint8_t * buffer) {
    static bool is_pin_inited = false;

    if (not is_pin_inited) {
        pinMode(motor_reg_data_pin, INPUT);
        pinMode(motor_reg_shift_pin, OUTPUT);
        pinMode(motor_reg_latch_pin, OUTPUT);
        digitalWrite(motor_reg_latch_pin, LOW);
    }

    digitalWrite(motor_reg_latch_pin, LOW);

    for (auto i = 0; i < 8; ++i) {
        buffer[i] = digitalRead(motor_reg_data_pin);

        digitalWrite(motor_reg_shift_pin, HIGH);
        digitalWrite(motor_reg_shift_pin, LOW);
    }

    digitalWrite(motor_reg_latch_pin, HIGH);

}

inline bool isSettingsResetPressed() { return digitalRead(settings_reset_pin) == LOW; }
#define TIME_TO_COVER_TIMINGS_RESET 5000  // ms
#define TIME_TO_SETTINGS_RESET 15000      // ms

void loop() {
    if (working_mode == ProcessingMqttCover) {
        // Проверяем подключение к WiFi
        if (wifiConnectionProcess(settings.wifi)) {
            // Подключение к WiFi установлено
            if (mqttConnectionWrapper->isConnected()) {
                // Подключение к MQTT установлено
                mqttClient.loop();
            } else {
                mqttConnectionWrapper->connect();
            }
        }

        read_sensors(sensor_buffer);

        static bool isCoverReady[4] = { false, false, false, false };
        for (auto i : window_indexes) {
            if (not isCoverReady[i]) {
                coverPreparer[i]->prepare();
                if (coverPreparer[i]->isPrepared()) {
                    Serial.printf("Cover %i is ready!\n", i);
                    isCoverReady[i] = true;

                    auto result = coverPreparer[i]->getResult();
                    coverTimingsManager[i]->saveTimeToOpen({ SPEED, result.timeToOpen });
                    coverTimingsManager[i]->saveTimeToClose({ SPEED, result.timeToClose });

                    if (result.lastState == CoverState::Closed)
                        covers[i]->setCurrentPosition(covers[i]->getClosedPosition());
                    else if (result.lastState == CoverState::Opened)
                        covers[i]->setCurrentPosition(covers[i]->getOpenedPosition());
                    else {
                        Serial.println("Unknown start state after prepare!");
                    }

                    if (covers[i]->getTargetPosition() == -1) covers[i]->setTargetPosition(covers[i]->getCurrentPosition());

                    coverMovementProcessor[i] = std::make_shared<PositionCoverMovementProcessor>(
                            covers[i], motor[i], sensors[i],
                            result.timeToOpen, result.timeToClose);

                    readyObserver[i]->setCoverReady();
                }
            } else {
                coverMovementProcessor[i]->process();
            }
        }

        write_motor_control(motor_buffer);

        static bool wasSettingsResetPressed = false;
        static unsigned long resetTimePressed;
        if (isSettingsResetPressed()) {
            if (wasSettingsResetPressed) {
                if (millis() - resetTimePressed > TIME_TO_SETTINGS_RESET) {
                    Serial.println("TIME_TO_SETTINGS_RESET...");
                    EEPROM.begin(EEPROM_SIZE);
                    EEPROM.put(Offsets::Settings, false);
                    EEPROM.end();
                    ESP.reset();
                }
            } else {
                resetTimePressed = millis();
                wasSettingsResetPressed = true;
                Serial.println("Pressed reset settings button...");
            }
        } else if (wasSettingsResetPressed) {
            auto time_passed = millis() - resetTimePressed;
            wasSettingsResetPressed = false;
            Serial.printf("Released reset settings button after %lu...\n", time_passed);
            if (time_passed > TIME_TO_COVER_TIMINGS_RESET) {
                Serial.println("TIME_TO_COVER_TIMINGS_RESET...");
                for (auto i : window_indexes) {
                    coverTimingsManager[i]->saveTimeToOpen({ 0, 0 });
                    coverTimingsManager[i]->saveTimeToClose({ 0, 0 });
                }
                if (WiFi.status() == WL_CONNECTED) WiFi.disconnect(true);
                ESP.reset();
            }
        }

    } else if (working_mode == HostingWebSettings) {
        server->handleClient();
    }
}

