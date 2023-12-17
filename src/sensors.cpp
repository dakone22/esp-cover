#include <Arduino.h>

#include "sensors.h"


CoverSensors::CoverSensors(uint8_t closed_sensor_pin, uint8_t opened_sensor_pin, int active_value)
        : _closed_sensor_pin(closed_sensor_pin), _opened_sensor_pin(opened_sensor_pin), _active_value(active_value) {
    pinMode(_closed_sensor_pin, INPUT);
    pinMode(_opened_sensor_pin, INPUT);
}

BufferedCoverSensors::BufferedCoverSensors(const uint8_t *buffer, uint8_t closed_sensor_index,
                                           uint8_t opened_sensor_index, int active_value)
        : _buffer(buffer), _closed_sensor_index(closed_sensor_index), _opened_sensor_index(opened_sensor_index), _active_value(active_value) {
}
