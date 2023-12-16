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
