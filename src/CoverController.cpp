#include "CoverController.h"

CoverController::CoverController(std::shared_ptr<IPositionCover> cover)
        : _cover(std::move(cover)) { }
