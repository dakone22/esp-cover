#ifndef MOTOR_H
#define MOTOR_H

class IMotorController {
public:
    virtual void rotateOpening() = 0;
    virtual void rotateClosing() = 0;
    virtual void stop() = 0;
};

class ISpeedableMotorController : public IMotorController {
public:
    virtual void setSpeed(int value) = 0;
    virtual int getCurrentSpeed() = 0;
    virtual int getMinSpeed() = 0;
    virtual int getMaxSpeed() = 0;
};

#endif