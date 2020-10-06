/*
 * Axis.cpp
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#include <Axis.h>
#include "event_groups.h"
#include <cmath>

Axis::Axis(	size_t xSize,
        DigitalIOPin ioStep,
        DigitalIOPin ioDirection,
        DigitalIOPin ioOriginSW,
        DigitalIOPin ioLimitSW,
        StepStarter_t start,
        StepStopper_t stop
) : xSizeInMM{ xSize }, ioStep{ ioStep }, ioDirection{ ioDirection }, ioOriginSW{ ioOriginSW }, ioLimitSW{ ioLimitSW }, start{ start }, stop{ stop } {
    ioStep.write(true);
    xTaskCreate(prvAxisTask, nullptr, configMINIMAL_STACK_SIZE, this, tskIDLE_PRIORITY + 1UL, nullptr);
}

void Axis::startMove(bool bIsRelative, int32_t xStepsToMove, float fStepsPerSecond) {
    if (!bIsRelative)
        xStepsToMove -= xCurrentPosition;

    if (xStepsToMove == 0)
        return;
    else if (xStepsToMove > 0)
        setDirection(kTowardsLimit);
    else if (xStepsToMove < 0)
        setDirection(kTowardsOrigin);

    xNumberOfSteps = xStepsRemaining = abs(xStepsToMove);

    start(fStepsPerSecond);

    xSemaphoreTake(xMoveComplete, portMAX_DELAY);
}

void Axis::endMove() {
    stop();

    portBASE_TYPE xHigherPriorityWoken = pdFALSE;
    xSemaphoreGiveFromISR(xMoveComplete, &xHigherPriorityWoken);
    portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void Axis::step() {
    ioStep.write(false);

    --xStepsRemaining;
    Direction direction = this->getDirection();

    if (direction == kTowardsOrigin) {
        --xCurrentPosition;
        if (ioOriginSW.read()) {
            if (xMaximumPosition == kPositionUnknown)
                xCurrentPosition = 0;
            endMove();
        } else if (!xStepsRemaining)
            endMove();
    } else if (direction == kTowardsLimit) {
        ++xCurrentPosition;

        if (ioLimitSW.read()) {
            if (xMaximumPosition == kPositionUnknown)
                xMaximumPosition = xCurrentPosition.load() - 1;
            endMove();
        } else if (!xStepsRemaining)
            endMove();
    }

    ioStep.write(true);
}

bool Axis::obstructed() const {
    return ioOriginSW.read() && ioLimitSW.read();
}

void Axis::enqueueMove(Move const & message) {
    xMoveQueue.push_back(message, portMAX_DELAY);
}

[[nodiscard]] Axis::Move Axis::dequeueMove() {
    return xMoveQueue.pop_front();
}

float Axis::calibrateStepsPerMM() {
    startMove(Move::Relative, INT32_MIN);
    startMove(Move::Relative, INT32_MAX);
    startMove(Move::Absolute, 0);

    return static_cast<float>(xMaximumPosition) / xSizeInMM;
}

void Axis::setDirection(Direction direction) {
    ioDirection.write(direction);
}

[[nodiscard]] Axis::Direction Axis::getDirection() const {
    return static_cast<Direction>(ioDirection.read());
}

void Axis::prvAxisTask(void* pvParameters) {
    static EventGroupHandle_t xEventGroup{ xEventGroupCreate() };
    static EventBits_t uxBitsToWaitFor{ 0 };
    static size_t xTasksCreated{ 0 };
    size_t xTaskID = xTasksCreated++;
    uxBitsToWaitFor = (uxBitsToWaitFor << 1) | 1;
    Axis& axis = *reinterpret_cast<Axis*>(pvParameters);

    vTaskDelay(10); // Let pins stabilise

    while (axis.obstructed());

    float const fStepsPerMM = axis.calibrateStepsPerMM();

    while (true) {
        Move move = axis.dequeueMove();
        xEventGroupSync(xEventGroup, 1 << xTaskID, uxBitsToWaitFor, portMAX_DELAY);
        axis.startMove(move.isRelative, move.fDistanceInMM * fStepsPerMM, move.xStepsPerSecond);
    }
}
