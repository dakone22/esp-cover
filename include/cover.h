#ifndef COVER_H
#define COVER_H

#include <memory>

enum class CoverState {
    Opened,
    Closed,
    Opening,
    Closing,
    Unknown,
};


class IStateCover {
public:
    virtual void open() = 0;
    virtual void close() = 0;
    virtual void stop() = 0;

    virtual CoverState getState() = 0;

    virtual ~IStateCover() = default;
};

class IPositionCover : public IStateCover {
public:
    static const int UnknownPosition = -1;

    virtual void setTargetPosition(int value) = 0;
    virtual int getTargetPosition() = 0;

    virtual void setCurrentPosition(int value) = 0;
    virtual int getCurrentPosition() = 0;

    virtual int getOpenedPosition() = 0;
    virtual int getClosedPosition() = 0;

    inline void open() override { setTargetPosition(getOpenedPosition()); };
    inline void close() override { setTargetPosition(getClosedPosition()); };
    inline void stop() override { setTargetPosition(getCurrentPosition()); setCurrentPosition(getCurrentPosition()); };

    CoverState getState() override;

    inline bool isTargetToOpen() {
        if (getOpenedPosition() > getClosedPosition()) {
            return getTargetPosition() > getCurrentPosition();
        } else {
            return getTargetPosition() < getCurrentPosition();
        }
    }


    inline bool isTargetToClose() {
        if (getOpenedPosition() > getClosedPosition()) {
            return getTargetPosition() < getCurrentPosition();
        } else {
            return getTargetPosition() > getCurrentPosition();
        }
    }
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

    inline void setTargetPosition(int value) override { targetPosition = value; }
    inline int getTargetPosition() override { return targetPosition; }

    inline void setCurrentPosition(int value) override { currentPosition = value; }
    inline int getCurrentPosition() override { return currentPosition; }

    inline int getOpenedPosition() override { return openedPosition; }
    inline int getClosedPosition() override { return closedPosition; }
};

#endif