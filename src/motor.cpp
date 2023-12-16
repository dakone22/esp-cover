#include "Arduino.h"

#include "motor.h"

void MotorDriverController::setSpeed(int speed) {
    assert(_min_speed <= speed and speed <= _max_speed);
    _current_speed = speed;

    auto analog_value = map(_current_speed, _min_speed, _max_speed, 0, MAX_ANALOG_VALUE);
    analogWrite(_pwm_speed_pin, analog_value);
}

MotorDriverController::MotorDriverController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_speed_pin,
                                             int min_speed, int max_speed)
        : _in1_pin(in1_pin), _in2_pin(in2_pin), _pwm_speed_pin(pwm_speed_pin),
          _min_speed(min_speed), _max_speed(max_speed) {
    pinMode(_in1_pin, OUTPUT);
    pinMode(_in2_pin, OUTPUT);
    pinMode(_pwm_speed_pin, OUTPUT);
    setSpeed(_max_speed);
}
