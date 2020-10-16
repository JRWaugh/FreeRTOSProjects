
/*
 * Axis.cpp
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#include <Axis.h>
#include <cmath>

Axis::Axis(
        uint8_t ucOriginDirection,
        DigitalIOPin&& ioStep,
        DigitalIOPin&& ioDirection,
        MoveBeginCallback onMoveBegin,
        MoveEndCallback onMoveEnd)
: ucOriginDirection{ ucOriginDirection }, ioStep{ ioStep }, ioDirection{ ioDirection }, onMoveBegin{ onMoveBegin }, onMoveEnd{ onMoveEnd } {
    ioStep.write(true);
    xTaskCreate(prvStepperTask, nullptr, 78, this, tskIDLE_PRIORITY + 1UL, &xTaskHandle);
}

Axis::~Axis() {
    // This won't actually make things function properly if axes are deleted at any point, but it's the thought that counts.
    --Axis::uxAxesCreated;
    endMove();
    vTaskDelete(xTaskHandle);
    vEventGroupDelete(xEventGroup);
}

void Axis::move(int32_t xStepsToMove, float fStepsPerSecond) {
    if (xStepsToMove == Move::Origin)
        xStepsToMove = 0 - xCurrentPosition;

    if (xStepsToMove == 0)
        return;
    else if (xStepsToMove > 0)
        ioDirection.write(!ucOriginDirection);
    else if (xStepsToMove < 0)
        ioDirection.write(ucOriginDirection);
    xStepsRemaining = abs(xStepsToMove);

    xEventGroupWaitBits(xEventGroup, 1UL << Enabled, pdFALSE, pdTRUE, portMAX_DELAY);
    xEventGroupClearBits(xEventGroup, 1UL << (uxID + Stopped));
    isStepping = true;
    onMoveBegin(fStepsPerSecond);
    waitForMoveEnd(portMAX_DELAY);
}

void Axis::step() {
    Axis::Stepping = this;

    ioStep.write(false);

    if (xStepsRemaining && --xStepsRemaining == 0)
        endMove();

    if (ioDirection.read() == ucOriginDirection)
        --xCurrentPosition;
    else
        ++xCurrentPosition;


    ioStep.write(true);
}

void Axis::calibrate() {
    move(INT32_MIN);
    move(INT32_MAX);
    xEventGroupSetBits(xEventGroup, 1UL << (uxID + Calibrated));
}

[[nodiscard]] int32_t Axis::getCurrentPosition() const {
    return xCurrentPosition;
}

[[nodiscard]] int32_t Axis::getMaximumPosition() const {
    return xMaximumPosition;
}

[[nodiscard]] size_t Axis::getID() const {
    return uxID;
}

EventBits_t Axis::waitForMoveEnd(TickType_t xTicksToWait) const {
    return xEventGroupWaitBits(xEventGroup, 1UL << (uxID + Stopped), pdFALSE, pdTRUE, xTicksToWait);
}

EventBits_t Axis::waitForCalibration(TickType_t xTicksToWait) const {
    return xEventGroupWaitBits(xEventGroup, 1UL << (uxID + Calibrated), pdFALSE, pdTRUE, xTicksToWait);
}

BaseType_t Axis::enqueueMove(Move const & message) {
    return xMoveQueue.push_back(message, portMAX_DELAY);
}

[[nodiscard]] Move Axis::dequeueMove() {
    return xMoveQueue.pop_front();
}

void Axis::setOriginDirection(uint8_t ucOriginDirection) {
    if (this->ucOriginDirection != ucOriginDirection)
        xCurrentPosition = xMaximumPosition - xCurrentPosition;
    this->ucOriginDirection = ucOriginDirection;
}

void Axis::endMove() {
    if (isStepping) {
        isStepping = false;
        onMoveEnd();

        if (xMaximumPosition == kPositionUnknown) {
            if (ioDirection.read() == ucOriginDirection) {
                xCurrentPosition = 0;
            } else {
                xMaximumPosition = xCurrentPosition.load();
            }
        }
        portBASE_TYPE xHigherPriorityWoken = pdFALSE;
        xEventGroupSetBitsFromISR(xEventGroup, 1UL << (uxID + Stopped), &xHigherPriorityWoken);
        portEND_SWITCHING_ISR(xHigherPriorityWoken);
    }
}

void Axis::prvStepperTask(void* pvParameters) {
    static EventBits_t uxBitsToWaitFor{ 0 };
    Axis& axis = *reinterpret_cast<Axis*>(pvParameters);

    uxBitsToWaitFor |= 1UL << (axis.getID() + MoveReady);

    axis.calibrate();

    while (true) {
        Move const move = axis.dequeueMove();
        xEventGroupSync(xEventGroup, 1UL << (axis.getID() + MoveReady), uxBitsToWaitFor, portMAX_DELAY);
        axis.move(move.xStepsToMove, move.fStepsPerSecond);
    }
}

EventGroupHandle_t Axis::xEventGroup{ xEventGroupCreate() };
Axis* Axis::Stepping{ nullptr };
size_t Axis::uxAxesCreated{ 0 };
