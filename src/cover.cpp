#include "cover.h"

CoverState IPositionCover::getState() {
    if (getCurrentPosition() == getOpenedPosition())
        return CoverState::Opened;

    if (getCurrentPosition() == getClosedPosition())
        return CoverState::Closed;

    if (isTargetToOpen())
        return CoverState::Opening;

    if (isTargetToClose())
        return CoverState::Closing;

    return CoverState::Unknown;
}

Cover::Cover(int closedPosition_, int openedPosition_)
        : closedPosition(closedPosition_), openedPosition(openedPosition_) { }
