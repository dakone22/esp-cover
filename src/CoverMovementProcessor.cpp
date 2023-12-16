#include <Arduino.h>

#include "CoverMovementProcessor.h"

PositionCoverMovementProcessor::PositionCoverMovementProcessor(std::shared_ptr<IPositionCover> cover_,
                                                               std::shared_ptr<IMotorController> motorController_,
                                                               std::shared_ptr<ICoverSensors> coverSensors_,
                                                               unsigned long timeToOpen_,
                                                               unsigned long timeToClose_)
        : cover(std::move(cover_)),
          motor(std::move(motorController_)),
          sensors(std::move(coverSensors_)),
          timeToOpen(timeToOpen_),
          timeToClose(timeToClose_) { }

void PositionCoverMovementProcessor::process() {
    static bool isMoving = false;

    if (sensors->isClosed()) {
        if (cover->getCurrentPosition() != cover->getClosedPosition()) {
            cover->setCurrentPosition(cover->getClosedPosition());
        }

        if (not cover->isTargetToOpen()) {
            motor->stop();
            isMoving = false;
        }
    } else if (sensors->isOpened()) {
        if (cover->getCurrentPosition() != cover->getOpenedPosition()) {
            cover->setCurrentPosition(cover->getOpenedPosition());
        }

        if (not cover->isTargetToClose()) {
            motor->stop();
            isMoving = false;
        }
    }

    if (cover->getCurrentPosition() == IPositionCover::UnknownPosition)
        Serial.println("Warning: Still unknown position, when shouldn't it!");

    static unsigned long startTime, estimatedEndTime;
    static int startPosition;

    if (cover->getTargetPosition() == -1) return;

    if (cover->getTargetPosition() == cover->getCurrentPosition()) {
        isMoving = false;
        motor->stop();
        return;
    }

    if (not isMoving) {
        isMoving = true;
        auto target_current_position_relative = abs(cover->getTargetPosition() - cover->getCurrentPosition());
        static const auto max_min_position_relative = abs(cover->getOpenedPosition() - cover->getClosedPosition());
        double timeModifier = (double) target_current_position_relative / max_min_position_relative;

        Serial.printf("x = |%i - %i|/|%i - %i| = %i/%i = %lf", cover->getTargetPosition(), cover->getCurrentPosition(),
                      cover->getOpenedPosition(), cover->getClosedPosition(), target_current_position_relative,
                      max_min_position_relative, timeModifier);

        if (cover->getTargetPosition() == cover->getOpenedPosition() or
            cover->getTargetPosition() == cover->getClosedPosition())
            timeModifier = 1;

        Serial.printf("=> x=%lf\n", timeModifier);

        startPosition = cover->getCurrentPosition();
        if (cover->isTargetToOpen()) {
            startTime = millis();
            estimatedEndTime = startTime + timeToOpen * timeModifier;
            motor->rotateOpening();
        } else if (cover->isTargetToClose()) {
            startTime = millis();
            estimatedEndTime = startTime + timeToClose * timeModifier;
            motor->rotateClosing();
        } else {
            Serial.print("WRONG STATE: ");
            Serial.print((int) cover->getState());
            isMoving = false;    // cancel
        }
    } else {
        unsigned long currentTime = millis();
        auto pos = map(currentTime, startTime, estimatedEndTime, startPosition, cover->getTargetPosition());
        cover->setCurrentPosition(pos);
        //Serial.println(pos);
        if (currentTime > estimatedEndTime and
            currentTime - estimatedEndTime > (cover->isTargetToOpen() ? timeToOpen : timeToClose)) {
            motor->stop();
            isMoving = false;
            Serial.printf(
                    "Warning: Moving (curTime=%lu;estimatedEndTime=%lu) more than calculated (%lu)! Force stop...\n",
                    currentTime, estimatedEndTime, (cover->isTargetToOpen() ? timeToOpen : timeToClose));
            cover->setCurrentPosition(cover->getTargetPosition());
            Serial.printf("pos: %i -> %i\n", cover->getCurrentPosition(), cover->getTargetPosition());
        }
    }
}
