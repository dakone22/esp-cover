#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

#include <Arduino.h>

const auto motor_controller_in_1_pin      = D6;
const auto motor_controller_in_2_pin      = D7;
const auto motor_controller_pwm_speed_pin = D8;

const auto sensor_opened_pin = D0;
const auto sensor_closed_pin = D5;

const auto settings_reset_pin = D4;

#endif