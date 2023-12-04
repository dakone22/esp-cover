#ifndef REALIZATIONS_H
#define REALIZATIONS_H


#include <Arduino.h>
#include <memory>

#include "cover.h"
#include "sensors.h"
#include "motor.h"

class CoverSensors : public ICoverSensors {
private:
    uint8_t _closed_sensor_pin;
    uint8_t _opened_sensor_pin;
    int _is_value;

public:
    CoverSensors(uint8_t closed_sensor_pin, uint8_t opened_sensor_pin, int is_value)
        : _closed_sensor_pin(closed_sensor_pin), _opened_sensor_pin(opened_sensor_pin), _is_value(is_value) {
        pinMode(_closed_sensor_pin, INPUT);
        pinMode(_opened_sensor_pin, INPUT);
    }
    
    bool isClosed() override { return digitalRead(_closed_sensor_pin) == _is_value; }
    bool isOpened() override { return digitalRead(_opened_sensor_pin) == _is_value; }
};

class MotorDriverController : public ISpeedableMotorController {
private:
    uint8_t _in_1_pin;
    uint8_t _in_2_pin;
    uint8_t _pwm_speed_pin;

    int _current_speed;
    int _min_speed;
    int _max_speed;

    void write_in_pins(int in_1, int in_2) {
        digitalWrite(_in_1_pin, in_1);
        digitalWrite(_in_2_pin, in_2);
    }

public:
    MotorDriverController(uint8_t in_1_pin, uint8_t in_2_pin, uint8_t pwm_speed_pin, int min_speed, int max_speed)
        : _in_1_pin(in_1_pin), _in_2_pin(in_2_pin), _pwm_speed_pin(pwm_speed_pin), 
          _min_speed(min_speed), _max_speed(max_speed) {
        pinMode(_in_1_pin, OUTPUT);
        pinMode(_in_2_pin, OUTPUT);
        pinMode(_pwm_speed_pin, OUTPUT);
    }

    void rotateOpening() override { write_in_pins(LOW, HIGH); };
    void rotateClosing() override { write_in_pins(HIGH, LOW); };
    void stop()          override { write_in_pins(LOW, LOW);  };

    void setSpeed(int value) override {
        _current_speed = value;
        analogWrite(_pwm_speed_pin, map(value, _min_speed, _max_speed, 0, 0xFF));
    };

    int getCurrentSpeed() override { return _current_speed; };
    int getMinSpeed() override { return _min_speed; };
    int getMaxSpeed() override { return _max_speed; };
};


class Cover : public IPositionCover {
private:
    int currentPosition = UnknownPosition;
    int targetPosition = UnknownPosition;
    int closedPosition;
    int openedPosition;

public:
    Cover(int closedPosition_, int openedPosition_)
        : closedPosition(closedPosition_), openedPosition(openedPosition_) { }

    void setTargetPosition(int value) { targetPosition = value; };
    int getTargetPosition()   { return targetPosition; };

    void setCurrentPosition(int value) { currentPosition = value; };
    int getCurrentPosition()  { return currentPosition; };

    int getOpenedPosition()   { return openedPosition; };
    int getClosedPosition()   { return closedPosition; };
};


class TimeToMoveRecorder {
private:
    static const long MS_TO_WAIT_INERTIA = 1000;

    long msToOpen = -1;
    long msToClose = -1;

    CoverState lastCoverState = CoverState::Unknown;

    enum State {
        Unknown, UnknownMoveDown, 
        StartRecordingMsToOpen, RecordedMsToOpen, 
        StartRecordingMsToClose, RecordedMsToClose, 
        Ready
    } state = Unknown;

    bool isWaiting = false;
    unsigned long msWaitedTime;
    
    void wait(long ms) {
        msWaitedTime = millis() + ms;
        isWaiting = true;
    }

public:
    long getMsToOpen() { return msToOpen; }
    long getMsToClose() { return msToClose; }
    CoverState getLastCoverState() { return lastCoverState; }

    bool isReady(std::shared_ptr<ICoverSensors> coverSensors, std::shared_ptr<ISpeedableMotorController> motorController) {
        while (true) {
            if (isWaiting) {
                if (millis() > msWaitedTime) 
                    isWaiting = false;
                else 
                    return false;
            }

            switch (state)
            {
            case Unknown:
                if (coverSensors->isOpened()) {
                    state = StartRecordingMsToClose;
                    break;
                }

                if (not coverSensors->isClosed()) motorController->rotateClosing();

                state = UnknownMoveDown;
            
            case UnknownMoveDown:
                if (not coverSensors->isClosed()) return false;
                
                motorController->stop();
                state = StartRecordingMsToOpen;
                
                wait(MS_TO_WAIT_INERTIA);

                break;

            case StartRecordingMsToOpen:
                state = RecordedMsToOpen;
                msToOpen = millis();
                motorController->rotateOpening();
            
            case RecordedMsToOpen:
                if (not coverSensors->isOpened()) return false;

                motorController->stop();
                msToOpen = millis() - msToOpen;

                state = msToClose == -1 ? StartRecordingMsToClose : Ready;
                lastCoverState = CoverState::Opened;
                
                wait(MS_TO_WAIT_INERTIA);
                
                break;

            case StartRecordingMsToClose:
                state = RecordedMsToClose;
                msToClose = millis();
                motorController->rotateClosing();
            
            case RecordedMsToClose:
                if (not coverSensors->isClosed()) return false;

                motorController->stop();
                msToClose = millis() - msToClose;

                state = msToOpen == -1 ? StartRecordingMsToOpen : Ready;
                lastCoverState = CoverState::Closed;

                wait(MS_TO_WAIT_INERTIA);
                
                break;
            
            case Ready:
                return true;
            
            default:
                return false; // uknown state?
            }
        }
    }
};

#endif