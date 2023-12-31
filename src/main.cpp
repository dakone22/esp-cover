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

// Идентификатор клиента на основе идентификатора чипа ESP
auto client_id = "esp8266_" + String(ESP.getChipId());

#ifdef DEBUG_SERIAL_OUT
const unsigned long BAUDRATE = 921600;
#endif

// Инициализация WiFi-клиента и MQTT-клиента
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Объявляем общие указатели для различных компонентов
std::shared_ptr<ICoverSensors> sensors = nullptr;
std::shared_ptr<ISpeedableMotorController> motor = nullptr;
std::shared_ptr<IPositionCover> cover = nullptr;
std::shared_ptr<IMqttConnectionWrapper> mqttConnectionWrapper = nullptr;
std::shared_ptr<IMqttCallbackWrapper> mqttCallbackWrapper = nullptr;
std::shared_ptr<IReadyObserver> readyObserver = nullptr;
std::shared_ptr<ICoverTimingsManager> coverTimingsManager = nullptr;
std::shared_ptr<ICoverPreparer> coverPreparer = nullptr;
std::shared_ptr<ICoverMovementProcessor> coverMovementProcessor = nullptr;

// Объявление настроек и режима работы
Settings settings;
enum Mode { HostingWebSettings, ProcessingMqttCover } working_mode;

std::shared_ptr<ESP8266WebServer> server;

// Обработка корневой конечной точки для веб-настроек
void handleRoot() {
    // HTML-форма для настройки параметров WiFi и MQTT
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

// Обработка конечной точки сохранения для сохранения настроек
void handleSave() {
    settings.valid = true;  // Отметить настройки как действительные

    // Разбор и сохранение настроек WiFi и MQTT

    // WiFi настройки
    String ssid = server->arg("ssid");
    String pass = server->arg("pass");
    if (ssid.length() > MAX_STRING_SIZE or pass.length() > MAX_STRING_SIZE) {
        server->send(400, "text/plain", "Error: SSID or password too long");
        return;
    }
    strncpy(settings.wifi.ssid, ssid.c_str(), MAX_STRING_SIZE);
    strncpy(settings.wifi.pass, pass.c_str(), MAX_STRING_SIZE);

    // MQTT настройки
    String host = server->arg("host");
    if (host.length() > MAX_STRING_SIZE) {
        server->send(400, "text/plain", "Error: MQTT host too long");
        return;
    }
    settings.mqtt.port = server->arg("port").toInt();
    String login = server->arg("login");
    String mqtt_pass = server->arg("mqtt_pass");
    String topic_prefix = server->arg("topic_prefix");
    if (login.length() > MAX_STRING_SIZE or mqtt_pass.length() > MAX_STRING_SIZE or topic_prefix.length() > MAX_STRING_SIZE) {
        server->send(400, "text/plain", "Error: MQTT login, password, or with_prefix prefix too long");
        return;
    }
    strncpy(settings.mqtt.host, host.c_str(), MAX_STRING_SIZE);
    strncpy(settings.mqtt.login, login.c_str(), MAX_STRING_SIZE);
    strncpy(settings.mqtt.pass, mqtt_pass.c_str(), MAX_STRING_SIZE);
    strncpy(settings.mqtt.topic_prefix, topic_prefix.c_str(), MAX_STRING_SIZE);

    // Сохранение настроек в EEPROM
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(Offsets::Settings, settings);
    EEPROM.commit();
    EEPROM.end();

    server->send(200, "text/plain", "Settings saved. Rebooting...");
    delay(1000);
    ESP.reset();
}

// Класс для управления топиками MQTT
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
    explicit Topics(const char * prefix) : _prefix(prefix) {
        for (int i : { 0, 1, 2, 3 }) {
            _topic_position[i] = with_prefix(i, mqtt_topic_position);
            _topic_state[i] = with_prefix(i, mqtt_topic_state);
            _topic_availability[i] = with_prefix(i, mqtt_topic_availability);
            _topic_set[i] = with_prefix(i, mqtt_topic_set);
            _topic_set_position[i] = with_prefix(i, mqtt_topic_set_position);

        }
    }

    // Getter methods for MQTT topics

    inline const char * getTopicPosition(int i) const { return _topic_position[i].c_str(); }
    inline const char * getTopicState(int i) const { return _topic_state[i].c_str(); }
    inline const char * getTopicAvailability(int i) const { return _topic_availability[i].c_str(); }
    inline const char * getTopicSet(int i) const { return _topic_set[i].c_str(); }
    inline const char * getTopicSetPosition(int i) const { return _topic_set_position[i].c_str(); }
};

std::shared_ptr<Topics> _topics;

// Функция для публикации состояния и положения шторы в MQTT
void publish_state_and_position_func(CoverState state, int position) {
    if (not mqttClient.connected()) return;

    static int lastPosition;
    if (position != lastPosition) {
        mqttClient.publish(_topics->getTopicPosition(1), String(position).c_str());
        lastPosition = position;
    }

    static CoverState lastState;
    if (state != lastState) {
        lastState = state;
        switch (state) {
            case CoverState::Unknown:
            case CoverState::Opened:
                mqttClient.publish(_topics->getTopicState(1), mqtt_topic_state_payload_opened);
                break;
            case CoverState::Closed:
                mqttClient.publish(_topics->getTopicState(1), mqtt_topic_state_payload_closed);
                break;
            case CoverState::Opening:
                mqttClient.publish(_topics->getTopicState(1), mqtt_topic_state_payload_opening);
                break;
            case CoverState::Closing:
                mqttClient.publish(_topics->getTopicState(1), mqtt_topic_state_payload_closing);
                break;

            default:
                break;
        }
    }
}


#define SPEED 255

// Функция установки компонентов шторы
void setup_cover() {
    sensors = std::make_shared<CoverSensors>(sensor_closed_pin,
                                             sensor_opened_pin,
                                             LOW);
#ifdef DEBUG_SERIAL_OUT
    Serial.printf("CLOSED: %d; OPENED: %d\n", sensors->isClosed(), sensors->isOpened());
#endif

    motor = std::make_shared<MotorDriverController>(motor_controller_in_1_pin,
                                                    motor_controller_in_2_pin,
                                                    motor_controller_pwm_speed_pin, 0, 255);
    motor->setSpeed(SPEED);

    cover = std::make_shared<MqttCover>(position_closed, position_open, publish_state_and_position_func);

    readyObserver = std::make_shared<ReadyObserver>([]() {
#ifdef DEBUG_SERIAL_OUT
        Serial.printf("Sending available at \"%s\"! pos:%i; connected:%i\n", _topics->getTopicAvailability(1), cover->getCurrentPosition(), mqttClient.connected());
#endif
        mqttClient.publish(_topics->getTopicAvailability(1), mqtt_topic_availability_payload_available);
        cover->setCurrentPosition(cover->getCurrentPosition());
    });

    coverTimingsManager = std::make_shared<CoverTimingsManager>();

    coverPreparer = std::make_shared<CoverPreparer>(sensors, motor, coverTimingsManager->loadTimeToOpen().value_or(-1),
                                                    coverTimingsManager->loadTimeToClose().value_or(-1));
}

// Функция для установки MQTT компонентов
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
#ifdef DEBUG_SERIAL_OUT
            Serial.printf("mqtt connected!%i\n", mqttClient.connected());
#endif
            readyObserver->setMqttConnected(true);
        } else {
#ifdef DEBUG_SERIAL_OUT
            Serial.print("failed, error code: ");
            Serial.print(mqttClient.state());
            Serial.println("!");
#endif
        }
    });
    mqttConnectionWrapper->setOnDisconnectionFunc([]() {
#ifdef DEBUG_SERIAL_OUT
        Serial.print("Mqtt Disconnected!");
#endif
        readyObserver->setMqttConnected(false);
    });

    TopicHandlers topic_handlers = {
            { _topics->getTopicSetPosition(1), mqtt_topic_set_position_handler },
            { _topics->getTopicSet(1),         mqtt_topic_set_handler },
    };
    mqttCallbackWrapper = std::make_shared<MqttCallbackWrapper>(
            mqttClient,
            std::make_shared<CoverController>(cover),
            topic_handlers);
}

// Функция настройки для инициализации компонентов в зависимости от режима работы
void setup() {
#ifdef DEBUG_SERIAL_OUT
    // region Настройка COM-порта
    Serial.begin(BAUDRATE);
    Serial.println();
    Serial.println("MQTT COVER FOR ESP8266");
    // endregion
#endif

    // Задержка на 5 секунд для инициализации
    delay(5000);

    // Загрузка сохраненных настроек
    settings = loadSettings();
    if (settings.valid) {
        // Если настройки верны, обработайте сообщение MQTT
        working_mode = Mode::ProcessingMqttCover;

        _topics = std::make_shared<Topics>(settings.mqtt.topic_prefix);
        setup_cover();
        setup_mqtt();

        pinMode(settings_reset_pin, INPUT_PULLUP);
    } else {
        // Если настройки не действительны, запускаем веб-страницу для настроек
        server = std::make_shared<ESP8266WebServer>();
        WiFi.softAP(client_id.c_str(), nullptr);

#ifdef DEBUG_SERIAL_OUT
        Serial.printf("Starting wifi \"%s\" is %i\n", client_id.c_str(), WiFi.softAP(client_id.c_str(), nullptr));
#endif

        server->on("/", HTTP_GET, handleRoot);
        server->on("/save", HTTP_POST, handleSave);
        server->begin();

#ifdef DEBUG_SERIAL_OUT
        Serial.print("Starting server at ");
        Serial.print(WiFi.softAPIP());
        Serial.print(":");
        Serial.println(server->getServer().port());
#endif

        working_mode = Mode::HostingWebSettings;
    }
}

// Функция проверки нажатия кнопки сброса настроек
inline bool isSettingsResetPressed() { return digitalRead(settings_reset_pin) == LOW; }

// Константы времени для сброса таймингов и настроек шторы
#define TIME_TO_COVER_TIMINGS_RESET 5000  // ms
#define TIME_TO_SETTINGS_RESET 15000      // ms

// Функция основного цикла
void loop() {
    if (working_mode == ProcessingMqttCover) {
        // Обработка режима шторы MQTT

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

        static bool isCoverReady = false;
        if (not isCoverReady) {
            coverPreparer->prepare();
            if (coverPreparer->isPrepared()) {
#ifdef DEBUG_SERIAL_OUT
                Serial.println("Cover is ready!");
#endif
                isCoverReady = true;

                auto result = coverPreparer->getResult();
                coverTimingsManager->saveTimeToOpen({ SPEED, result.timeToOpen });
                coverTimingsManager->saveTimeToClose({ SPEED, result.timeToClose });

                if (result.lastState == CoverState::Closed)
                    cover->setCurrentPosition(cover->getClosedPosition());
                else if (result.lastState == CoverState::Opened)
                    cover->setCurrentPosition(cover->getOpenedPosition());
                else {
#ifdef DEBUG_SERIAL_OUT
                    Serial.println("Unknown start state after prepare!");
#endif
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

        static bool wasSettingsResetPressed = false;
        static unsigned long resetTimePressed;
        if (isSettingsResetPressed()) {
            if (wasSettingsResetPressed) {
                if (millis() - resetTimePressed > TIME_TO_SETTINGS_RESET) {
#ifdef DEBUG_SERIAL_OUT
                    Serial.println("TIME_TO_SETTINGS_RESET...");
#endif
                    EEPROM.begin(EEPROM_SIZE);
                    EEPROM.put(Offsets::Settings, false);
                    EEPROM.end();
                    ESP.reset();
                }
            } else {
                resetTimePressed = millis();
                wasSettingsResetPressed = true;
#ifdef DEBUG_SERIAL_OUT
                Serial.println("Pressed reset settings button...");
#endif
            }
        } else if (wasSettingsResetPressed) {
            auto time_passed = millis() - resetTimePressed;
            wasSettingsResetPressed = false;
#ifdef DEBUG_SERIAL_OUT
            Serial.printf("Released reset settings button after %lu...\n", time_passed);
#endif
            if (time_passed > TIME_TO_COVER_TIMINGS_RESET) {
#ifdef DEBUG_SERIAL_OUT
                Serial.println("TIME_TO_COVER_TIMINGS_RESET...");
#endif
                coverTimingsManager->saveTimeToOpen({ 0, 0 });
                coverTimingsManager->saveTimeToClose({ 0, 0 });
                if (WiFi.status() == WL_CONNECTED) WiFi.disconnect(true);
                ESP.reset();
            }
        }

    } else if (working_mode == HostingWebSettings) {
        // Режим веб-страницы настроек
        server->handleClient();
    }
}

