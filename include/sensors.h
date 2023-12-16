#ifndef SENSORS_H
#define SENSORS_H

class ICoverSensors {
public:
    virtual bool isClosed() = 0;
    virtual bool isOpened() = 0;
    virtual ~ICoverSensors() = default;
};

class CoverSensors : public ICoverSensors {
private:
    uint8_t _closed_sensor_pin;
    uint8_t _opened_sensor_pin;
    int _active_value;

public:
    CoverSensors(uint8_t closed_sensor_pin, uint8_t opened_sensor_pin, int active_value);

    inline bool isClosed() override { return digitalRead(_closed_sensor_pin) == _active_value; }
    inline bool isOpened() override { return digitalRead(_opened_sensor_pin) == _active_value; }
};

#endif // SENSORS_H