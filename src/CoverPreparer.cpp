#include "CoverPreparer.h"

CoverPreparer::CoverPreparer(const std::shared_ptr<ICoverSensors> & coverSensors,
                             const std::shared_ptr<IMotorController> & motorController, unsigned long timeToOpen,
                             unsigned long timeToClose)
        : _coverSensors(coverSensors), _motorController(motorController), msToOpen(timeToOpen), msToClose(timeToClose) { }

CoverPrepareResult CoverPreparer::getResult() const { return { msToOpen, msToClose, lastCoverState }; }

void CoverPreparer::prepare() {
    while (true) {
        if (isWaiting) {
            if (millis() > msWaitedTime)
                isWaiting = false;
            else
                return;
        }

        switch (state) {
            case Unknown:
                if (_coverSensors->isOpened()) {
                    lastCoverState = CoverState::Opened;
                    state = msToClose != -1UL and msToOpen != -1UL ? Ready : StartRecordingMsToClose;
                    break;
                }

                if (not _coverSensors->isClosed()) _motorController->rotateClosing();

                state = UnknownMoveDown;

            case UnknownMoveDown:
                if (not _coverSensors->isClosed()) return;

                _motorController->stop();

                lastCoverState = CoverState::Closed;
                state = msToClose != -1UL and msToOpen != -1UL ? Ready : StartRecordingMsToOpen;

                setMsToWait(MS_TO_WAIT_INERTIA);

                break;

            case StartRecordingMsToOpen:
                state = RecordedMsToOpen;
                msToOpen = millis();
                _motorController->rotateOpening();

            case RecordedMsToOpen:
                if (not _coverSensors->isOpened()) return;

                _motorController->stop();
                msToOpen = millis() - msToOpen;
#ifdef DEBUG_SERIAL_OUT
                Serial.printf("Recorded ms to open: %lu\n", msToOpen);
#endif
                state = msToClose == -1UL ? StartRecordingMsToClose : Ready;
                lastCoverState = CoverState::Opened;

                setMsToWait(MS_TO_WAIT_INERTIA);

                break;

            case StartRecordingMsToClose:
                state = RecordedMsToClose;
                msToClose = millis();
                _motorController->rotateClosing();

            case RecordedMsToClose:
                if (not _coverSensors->isClosed()) return;

                _motorController->stop();
                msToClose = millis() - msToClose;
#ifdef DEBUG_SERIAL_OUT
                Serial.printf("Recorded ms to close: %lu\n", msToClose);
#endif
                state = msToOpen == -1UL ? StartRecordingMsToOpen : Ready;
                lastCoverState = CoverState::Closed;

                setMsToWait(MS_TO_WAIT_INERTIA);

                break;

            case Ready:
                return;
        }
    }
}
