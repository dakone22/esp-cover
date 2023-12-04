#ifndef SENSORS_H
#define SENSORS_H

class ICoverSensors {
public:
    virtual bool isClosed() = 0;
    virtual bool isOpened() = 0;
};

#endif // SENSORS_H