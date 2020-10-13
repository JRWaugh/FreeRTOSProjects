/*
 * Axis.cpp
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#include <Axis.h>
#include <cmath>
#include "timers.h"
[[nodiscard]] static bool isInterrupt() {
    return SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
}

static EventGroupHandle_t xEventGroup{ xEventGroupCreate() };

Axis::Axis(	size_t uxSizeInMM,
        uint8_t ucOriginDirection,
        DigitalIOPin&& ioStep,
        DigitalIOPin&& ioDirection,
        DigitalIOPin&& ioOriginSW,
        DigitalIOPin&& ioLimitSW,
        StepEnableCallback start,
        StepDisableCallback stop
) :
        uxSizeInMM{ uxSizeInMM }, ucOriginDirection{ ucOriginDirection },
        ioStep{ ioStep }, ioDirection{ ioDirection }, ioOriginSW{ ioOriginSW }, ioLimitSW{ ioLimitSW },
        startStepping{ start }, stopStepping{ stop } {
    ioStep.write(true);
    xTaskCreate(prvAxisTask, nullptr, 70, this, tskIDLE_PRIORITY + 1UL, &xTaskHandle);
}

Axis::~Axis() {
    onMoveComplete();
    vTaskDelete(xTaskHandle); // Will still have a memory leak because of the event group allocated in prvAsixTask, but oh well!
    vSemaphoreDelete(xMoveComplete);
    vEventGroupDelete(xEventGroup);
}

void Axis::move(bool isRelative, int32_t xStepsToMove, float fStepsPerSecond) {
    if (!isRelative)
        xStepsToMove -= xCurrentPosition;

    if (xStepsToMove == 0)
        return;
    else if (xStepsToMove > 0)
        ioDirection.write(!ucOriginDirection);
    else if (xStepsToMove < 0)
        ioDirection.write(ucOriginDirection);
    xStepsRemaining = abs(xStepsToMove);

    xEventGroupWaitBits(xEventGroup, uxEnableBit, pdFALSE, pdTRUE, portMAX_DELAY);
    startStepping(fStepsPerSecond);
    xSemaphoreTake(xMoveComplete, portMAX_DELAY);
}

void Axis::movef(bool isRelative, float fDistanceToMove, float fStepsPerSecond) {
    move(isRelative, std::round(fDistanceToMove * fStepsPerMM), fStepsPerSecond);
}

void Axis::step() {
    ioStep.write(false);

    --xStepsRemaining;

    if (ioDirection.read() == ucOriginDirection) {
        --xCurrentPosition;
        bool const isPressed = ucOriginDirection == Clockwise ? readOriginSwitch() : readLimitSwitch();
        if (isPressed) {
            if (xMaximumPosition == kPositionUnknown)
                xCurrentPosition = 0;
            onMoveComplete();
        } else if (xStepsRemaining == 0)
            onMoveComplete();
    } else {
        ++xCurrentPosition;
        bool const isPressed = ucOriginDirection == Clockwise ? readLimitSwitch() : readOriginSwitch();
        if (isPressed) {
            if (xMaximumPosition == kPositionUnknown)
                xMaximumPosition = xCurrentPosition.load() - 1;
            onMoveComplete();
        } else if (xStepsRemaining == 0)
            onMoveComplete();
    }

    ioStep.write(true);
}

void Axis::halt() {
    if (!isInterrupt())
        xEventGroupClearBits(xEventGroup, uxEnableBit);
    else
        xEventGroupClearBitsFromISR(xEventGroup, uxEnableBit);
}

void Axis::resume() {
    if (!isInterrupt())
        xEventGroupSetBits(xEventGroup, uxEnableBit);
    else {
        portBASE_TYPE xHigherPriorityWoken = pdFALSE;
        xEventGroupSetBitsFromISR(xEventGroup, uxEnableBit, &xHigherPriorityWoken);
        portEND_SWITCHING_ISR(xHigherPriorityWoken);
    }
}

void Axis::onMoveComplete() {
    stopStepping();

    portBASE_TYPE xHigherPriorityWoken = pdFALSE;
    xSemaphoreGiveFromISR(xMoveComplete, &xHigherPriorityWoken);
    portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

bool Axis::readOriginSwitch() {
    return ioOriginSW.read();
}

bool Axis::readLimitSwitch() {
    return ioLimitSW.read();
}

BaseType_t Axis::enqueueMove(Move const & message) {
    return xMoveQueue.push_back(message, portMAX_DELAY);
}

[[nodiscard]] Axis::Move Axis::dequeueMove() {
    return xMoveQueue.pop_front();
}

void Axis::calibrate() {
    move(Move::Relative, INT32_MIN);
    move(Move::Relative, INT32_MAX);
    move(Move::Absolute, 0);
    fStepsPerMM = static_cast<float>(xMaximumPosition) / uxSizeInMM;
}

void Axis::prvAxisTask(void* pvParameters) {
    static size_t uxTasksCreated{ 0 };
    static EventBits_t uxBitsToWaitFor{ 0 };

    size_t const uxTaskID = uxTasksCreated++;
    uxBitsToWaitFor |= 1 << uxTaskID;
    Axis& axis = *reinterpret_cast<Axis*>(pvParameters);

    axis.calibrate();

    while (true) {
        Move const move = axis.dequeueMove();
        xEventGroupSync(xEventGroup, 1 << uxTaskID, uxBitsToWaitFor, portMAX_DELAY);
        axis.movef(move.isRelative, move.fDistanceInMM, move.xStepsPerSecond);
    }
}
