#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "mqtt.h"
#include "realizations.h"

/**************************************************************************
 * Настройки проекта
 *************************************************************************/
#include "config.h"
const char * client_id = ("esp8266_" + String(ESP.getChipId())).c_str();

/**************************************************************************
 * Глобальные переменные
 *************************************************************************/
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

std::shared_ptr<ICoverSensors>             sensors;
std::shared_ptr<ISpeedableMotorController> motor;
std::shared_ptr<IPositionCover>            cover;

bool isCoverReady = false;

// Текущее состояние цифровых входов
// Последнее опубликованное состояние реле
// Здесь маленькая хитрость: публикация в топик происходит только при наличии изменений
// (то есть когда inputStatus1 != digitalRead()), то при первом подключении ничего не будет опубликовано
// Дабы форсировать события, проинициализируем переменные заведомо ложными данными, которых не будет при любом состоянии входов
char inputStatus1 = 2;

/**************************************************************************
 * Сервисные функции
 *************************************************************************/

// Подключение к WiFi точке доступа
bool wifiConnectionProcess()
{
  static enum State {
    Start, Connecting, Connected, LostConnection
  } state = Start;
  static bool isWaiting = false;
  static unsigned long nextTime;

  // Если подключение активно, то просто выходим и возвращаем true
  if (WiFi.status() == WL_CONNECTED) return true;
  
  while (true)
  {
    if (isWaiting) {
      if (millis() > nextTime) 
          isWaiting = false;
      else 
          return false;
    }

    switch (state)
    {
    
    case LostConnection:
      Serial.println("Lost connection, attempting again...");
      state = Start;

    case Start:
      // ... иначе пробуем подключиться к сети
      Serial.print("Connecting to WiFi AP ");
      Serial.println(wifi_ssid);

      // Настраиваем WiFi
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifi_ssid, wifi_pass);

      state = Connecting;
      break;

    case Connecting:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting...");

        isWaiting = true;
        nextTime = millis() + 500;
        
        return false;
      }

      // Подключение успешно установлено
      Serial.println(" ок");
      Serial.print("WiFi connected, obtained IP address: ");
      Serial.println(WiFi.localIP());
      
      state = Connected;
    
    case Connected:
      if (WiFi.status() != WL_CONNECTED) {
        state = LostConnection;
        break;
      }
      return true;

    default:
      break;
    }
  }
}

// Функция обратного вызова при поступлении входящего сообщения от брокера
void mqttOnIncomingMsg(char* topic, byte* payload, unsigned int length)
{
  // Для более корректного сравнения строк обрезаем пробелы с краев
  String _payload;
  for (unsigned int i = 0; i < length; i++) {
    _payload += String((char)payload[i]);
  };
  _payload.trim();

  // Вывод поступившего сообщения в лог, больше никакого смысла этот блок кода не несет, можно исключить
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.print(_payload.c_str());
  Serial.println();

  if (cover == nullptr) return;

  // Сравниваем с топиками
  String _topic(topic);
  if (_topic.equals(mqtt_topic_set_position)) {
    int pos = _payload.toInt();

    cover->setTargetPosition(pos);

  } else if (_topic.equals(mqtt_topic_set)) {
    if (_payload.equals(mqtt_topic_set_payload_open)) {
      cover->open();
    } else if (_payload.equals(mqtt_topic_set_payload_close)) {
      cover->close();
    } else if (_payload.equals(mqtt_topic_set_payload_stop)) {
      cover->stop();
    } else {
      Serial.println("Unknown payload: " + _payload);
    }

  } else {
    Serial.println("Failed to recognize incoming topic!");
  };
}

// Подключение к MQTT брокеру :: версия для PubSubClient от knolleary ("стандартная")
bool mqttConnected()
{
  if (mqttClient.connected()) return true;

  Serial.println("Connecting to MQTT broker...");
  // Настраиваем MQTT клиент
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttOnIncomingMsg);

  // Пробуем подключиться с LWT сообщением "offline"
  if (mqttClient.connect(client_id, mqtt_login, mqtt_pass,
                          // last will: send `not available` when disconnected
                          mqtt_topic_availability, 
                          MQTT::QoS::AtLeastOnce, 
                          MQTT::RetainMessage::Yes,
                          mqtt_topic_availability_payload_not_available)) {
    Serial.println("Connected to MQTT broker!");
    
    // Подписываемся на топики
    mqttClient.subscribe(mqtt_topic_set, mqtt_topic_set_qos);
    mqttClient.subscribe(mqtt_topic_set_position, mqtt_topic_set_position_qos);

    // Опубликуем текущие состояния реле
    // mqttClient.publish(mqtt_topic_position, String(cover->getCurrentPosition()).c_str());
    // mqttClient.publish(mqtt_topic_state, cover->getState() == CoverState::Unknown ? );
  } else {
    Serial.print("failed, error code: ");
    Serial.print(mqttClient.state());
    Serial.println("!");
  };
  return mqttClient.connected();
}

class MqttCover : public Cover {
private:
  void publish_position() {
    if (not mqttClient.connected()) return;
    
    mqttClient.publish(mqtt_topic_position, String(getCurrentPosition()).c_str());

    if (getState() != CoverState::Unknown) {
      switch (getState())
      {
      case CoverState::Opened:
        mqttClient.publish(mqtt_topic_state, mqtt_topic_state_payload_open);
        break;
      case CoverState::Closed:
        mqttClient.publish(mqtt_topic_state, mqtt_topic_state_payload_close);
        break;
      case CoverState::Opening:
        mqttClient.publish(mqtt_topic_state, mqtt_topic_state_payload_opening);
        break;
      case CoverState::Closing:
        mqttClient.publish(mqtt_topic_state, mqtt_topic_state_payload_closing);
        break;
      
      default:
        break;
      }
    }
  }

public:
  MqttCover(int closedPosition, int openedPosition) : Cover(closedPosition, openedPosition) { }

  void setCurrentPosition(int value) override {
    Cover::setCurrentPosition(value);
    publish_position();
  };
};

static unsigned long timeToOpen, timeToClose;

void process() {
  static bool isMoving = false;

  if (sensors->isClosed() and not cover->isTargetToOpen()) {
      motor->stop();
      isMoving = false;
      cover->setCurrentPosition(cover->getClosedPosition());
  } else if (sensors->isOpened() and not cover->isTargetToClose()) {
      motor->stop();
      isMoving = false;
      cover->setCurrentPosition(cover->getOpenedPosition());
  }

  if (cover->getCurrentPosition() == IPositionCover::UnknownPosition)
    Serial.println("Warning: Still unknown position, when shouldn't it!");

  static unsigned long startTime, estimatedEndTime;

  if (cover->getTargetPosition() == cover->getCurrentPosition()) {
    isMoving = false;
    motor->stop();
    return;
  }
  
  if (not isMoving) {
    isMoving = true;
    if (cover->isTargetToOpen()) {
      startTime = millis();
      estimatedEndTime = startTime + timeToOpen;
      motor->rotateOpening();
    } else if (cover->isTargetToClose()) {
      startTime = millis();
      estimatedEndTime = startTime + timeToClose;
      motor->rotateClosing();
    } else {
      Serial.print("WRONG STATE: ");
      Serial.print((int)cover->getState());
      isMoving = false;  // cancel
    }
  } else {
    static long openedPosition = cover->getOpenedPosition();
    static long closedPosition = cover->getClosedPosition();
    unsigned long currentTime = millis();
    cover->setCurrentPosition(map(currentTime, startTime, estimatedEndTime, openedPosition, closedPosition));
    if (currentTime > estimatedEndTime) {
      motor->stop();
      isMoving = false;
      Serial.println("Warning: Moving more than calculated! Force stop...");
      cover->setCurrentPosition(cover->isTargetToOpen() ? cover->getOpenedPosition() : cover->getClosedPosition());
    }
  }
}


/**************************************************************************
 * Основные функции
 *************************************************************************/

void setup() {
  // Настройка COM-порта
  Serial.begin(921600);
  Serial.println();
  Serial.println("MQTT COVER FOR ESP8266");

  sensors = std::make_shared<CoverSensors>(sensor_closed_pin, 
                                           sensor_opened_pin, 
                                           LOW);
  Serial.printf("CLOSED: %d; OPENED: %d", sensors->isClosed(), sensors->isOpened());

  motor = std::make_shared<MotorDriverController>(motor_controller_in_1_pin, 
                                                  motor_controller_in_2_pin,
                                                  motor_controller_pwm_speed_pin, 0, 255);
  motor->setSpeed(180);

  cover = std::make_shared<MqttCover>(position_closed, position_open);
  // cover->setCurrentPosition(position_open);

  delay(5000);
}

static TimeToMoveRecorder r;

void loop() {
  // Проверяем подключение к WiFi
  if (wifiConnectionProcess()) {
    // Подключение к WiFi установлено 
    if (mqttConnected()) {
      // Подключение к MQTT установлено 
      mqttClient.loop();
    };
  };

  if (not isCoverReady) {
    if (r.isReady(sensors, motor)) {
      Serial.println("Cover is ready!");
      isCoverReady = true;
      timeToOpen = r.getMsToOpen();
      timeToClose = r.getMsToClose();
      if (r.getLastCoverState() == CoverState::Closed)
        cover->setCurrentPosition(cover->getClosedPosition());
      else if (r.getLastCoverState() == CoverState::Opened)
        cover->setCurrentPosition(cover->getOpenedPosition());
      
      mqttClient.publish(mqtt_topic_availability, mqtt_topic_availability_payload_available);
    }
  } else {
    process();
  }
}

