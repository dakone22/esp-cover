#ifndef MOTOR_H
#define MOTOR_H

class IMotorController {
public:
    virtual void rotateOpening() = 0;
    virtual void rotateClosing() = 0;
    virtual void stop() = 0;
    virtual ~IMotorController() = default;
};

class ISpeedableMotorController : public IMotorController {
public:
    virtual void setSpeed(int value) = 0;
    virtual int getCurrentSpeed() = 0;

    virtual int getMinSpeed() = 0;
    virtual int getMaxSpeed() = 0;
};

class MotorDriverController : public ISpeedableMotorController {
private:
    const int MAX_ANALOG_VALUE = 0xFF;

    uint8_t _in1_pin;
    uint8_t _in2_pin;
    uint8_t _pwm_speed_pin;

    int _current_speed{};
    int _min_speed;
    int _max_speed;

    inline void write_in_pins(int in1, int in2) const {
        digitalWrite(_in1_pin, in1);
        digitalWrite(_in2_pin, in2);
    }

public:
    MotorDriverController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_speed_pin, int min_speed, int max_speed);

    inline void rotateOpening() override { write_in_pins(LOW, HIGH); }
    inline void rotateClosing() override { write_in_pins(HIGH, LOW); }
    inline void stop() override          { write_in_pins(LOW, LOW); }

    void setSpeed(int speed) override;
    inline int getCurrentSpeed() override { return _current_speed; };

    inline int getMinSpeed() override { return _min_speed; };
    inline int getMaxSpeed() override { return _max_speed; };
};

#endif