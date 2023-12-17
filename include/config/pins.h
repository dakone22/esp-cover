#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

#include <Arduino.h>

//const auto motor_controller_in_1_pin      = D6;
//const auto motor_controller_in_2_pin      = D7;
//const auto motor_controller_pwm_speed_pin = D8;
//
//const auto sensor_opened_pin = D0;
//const auto sensor_closed_pin = D5;

const unsigned char motor_controller_pwm_speed_pin[] = {D0, D1, D2, D3};

const auto motor_reg_data_pin = D5;
const auto motor_reg_shift_pin = D6;
const auto motor_reg_latch_pin = D7;

const auto sensor_reg_data_pin = TX;
const auto sensor_reg_shift_pin = D8;
const auto sensor_reg_latch_pin = RX;

const auto settings_reset_pin = D4;

#endif