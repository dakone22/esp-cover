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

class PwmSpeedableMotorController : public ISpeedableMotorController {
private:
    const int MAX_ANALOG_VALUE = 0xFF;

    uint8_t _pwm_speed_pin;

    int _current_speed{};
    int _min_speed;
    int _max_speed;
public:
    PwmSpeedableMotorController(uint8_t pwm_speed_pin, int min_speed, int max_speed)
            : _pwm_speed_pin(pwm_speed_pin), _min_speed(min_speed), _max_speed(max_speed) {
        pinMode(_pwm_speed_pin, OUTPUT);
        setSpeed(_max_speed);
    }

    void setSpeed(int speed) override;
    inline int getCurrentSpeed() override { return _current_speed; };

    inline int getMinSpeed() override { return _min_speed; };
    inline int getMaxSpeed() override { return _max_speed; };
};

class MotorDriverController : public PwmSpeedableMotorController {
private:
    uint8_t _in1_pin;
    uint8_t _in2_pin;


    inline void write_in_pins(int in1, int in2) const {
        digitalWrite(_in1_pin, in1);
        digitalWrite(_in2_pin, in2);
    }

public:
    MotorDriverController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_speed_pin, int min_speed, int max_speed);

    inline void rotateOpening() override { write_in_pins(LOW, HIGH); }
    inline void rotateClosing() override { write_in_pins(HIGH, LOW); }
    inline void stop() override          { write_in_pins(LOW, LOW); }

};

class BufferedMotorDriverController : public PwmSpeedableMotorController {
private:
    uint8_t * buffer;
    size_t _in1_index;
    size_t _in2_index;

    inline void write_in_buffer(int in1, int in2) const {
        buffer[_in1_index] = in1;
        buffer[_in2_index] = in2;
    }

public:
    BufferedMotorDriverController(uint8_t * buffer, size_t in1_index, size_t in2_index, uint8_t pwm_speed_pin, int min_speed, int max_speed);

    inline void rotateOpening() override { write_in_buffer(LOW, HIGH); }
    inline void rotateClosing() override { write_in_buffer(HIGH, LOW); }
    inline void stop() override          { write_in_buffer(LOW, LOW); }
};

#endif