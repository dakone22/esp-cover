#ifndef COVER_H
#define COVER_H

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
};

class IPositionCover : public IStateCover {
public:
    static const int UnknownPosition = -1;

    virtual void setTargetPosition(int value) = 0;
    virtual int getTargetPosition()           = 0;

    virtual void setCurrentPosition(int value) = 0;
    virtual int getCurrentPosition()           = 0;

    virtual int getOpenedPosition()  = 0;
    virtual int getClosedPosition()  = 0;

    void open()  override { setTargetPosition(getOpenedPosition()); };
    void close() override { setTargetPosition(getClosedPosition()); };
    void stop()  override { setTargetPosition(getCurrentPosition()); };
    
    CoverState getState()  override { 
        if (getCurrentPosition() == getOpenedPosition())
            return CoverState::Opened;

        if (getCurrentPosition() == getClosedPosition())
            return CoverState::Closed;
        
        if (getTargetPosition() > getCurrentPosition())
            return getOpenedPosition() > getClosedPosition() ? CoverState::Opening : CoverState::Closing;

        if (getTargetPosition() < getCurrentPosition())
            return getOpenedPosition() > getClosedPosition() ? CoverState::Closing : CoverState::Opening;

        return CoverState::Unknown;
    };

    bool isTargetToOpen() {
        return(getTargetPosition() > getCurrentPosition()) and (getOpenedPosition() > getClosedPosition());
    }

    bool isTargetToClose() {
        return(getTargetPosition() < getCurrentPosition()) and (getOpenedPosition() > getClosedPosition());
    }
};

#endif