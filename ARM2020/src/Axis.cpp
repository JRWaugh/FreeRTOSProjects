/*
 * Axis.cpp
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#include <Axis.h>
#include <cmath>

// Lazy copy-paste of isInterrupt function, since I didn't feel like writing a utilities file that wouldn't have much else in it.
[[nodiscard]] static bool isInterrupt() { return SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk; }
static EventGroupHandle_t xEventGroup{ xEventGroupCreate() };
static constexpr EventBits_t uxEnableBit{ 1UL << 23 };
size_t Axis::uxAxesCreated{ 0 };
EventBits_t Axis::uxBitsToWaitFor{ 0 };

Axis::Axis(	size_t uxSizeInMM,
        uint8_t ucOriginDirection,
        DigitalIOPin&& ioStep,
        DigitalIOPin&& ioDirection,
        DigitalIOPin&& ioLimitSW1,
        DigitalIOPin&& ioLimitSW2,
        StepEnableCallback start,
        StepDisableCallback stop)
// I know this is an extremely long line, but MCUXpresso does not handle auto-identation correctly unless you do this.
: uxTaskID{ Axis::uxAxesCreated++ }, uxSizeInMM{ uxSizeInMM }, ucOriginDirection{ ucOriginDirection }, ioStep{ ioStep }, ioDirection{ ioDirection }, ioLimitSW1{ ioLimitSW1 }, ioLimitSW2{ ioLimitSW2 }, startStepping{ start }, stopStepping{ stop } {
    Axis::uxBitsToWaitFor |= 1UL << uxTaskID | 1UL << (uxTaskID + 12);
    ioStep.write(true);
    xTaskCreate(prvAxisTask, nullptr, 70, this, tskIDLE_PRIORITY + 1UL, &xTaskHandle);
}

Axis::~Axis() {
    // This won't actually make things function properly if axes are deleted at any point, but it's the thought that counts.
    Axis::uxBitsToWaitFor &= ~(1UL << uxTaskID | 1UL << uxTaskID + 12);
    --Axis::uxAxesCreated;
    onMoveComplete();
    vTaskDelete(xTaskHandle);
    vSemaphoreDelete(xMoveComplete);
    vEventGroupDelete(xEventGroup);
}

void Axis::move(int32_t xStepsToMove, float fStepsPerSecond) {
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

void Axis::movef(float fDistanceToMove, float fStepsPerSecond) {
    move(std::round(fDistanceToMove * fStepsPerMM), fStepsPerSecond);
}

void Axis::step() {
    ioStep.write(false);

    --xStepsRemaining;

    if (ioDirection.read() == ucOriginDirection) {
        --xCurrentPosition;
        if (readOriginSwitch()) {
            if (xMaximumPosition == kPositionUnknown)
                xCurrentPosition = 0;
            onMoveComplete();
        } else if (xStepsRemaining == 0)
            onMoveComplete();
    } else {
        ++xCurrentPosition;
        if (readLimitSwitch()) {
            if (xMaximumPosition == kPositionUnknown)
                xMaximumPosition = xCurrentPosition.load() - 1;
            onMoveComplete();
        } else if (xStepsRemaining == 0)
            onMoveComplete();
    }

    ioStep.write(true);
}

void Axis::calibrate() {
    move(INT32_MIN);
    move(INT32_MAX);
    move(0 - xCurrentPosition);
    fStepsPerMM = static_cast<float>(xMaximumPosition) / uxSizeInMM;
    xEventGroupSetBits(xEventGroup, 1UL << (uxTaskID + 12));
}

EventBits_t Axis::waitForCalibration(TickType_t xTicksToWait) {
    auto test = xEventGroupGetBits(xEventGroup);
    return xEventGroupWaitBits(xEventGroup, uxBitsToWaitFor & 0xFFE000, pdFALSE, pdTRUE, xTicksToWait);
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

bool Axis::readOriginSwitch() {
    return ucOriginDirection == Clockwise ? ioLimitSW1.read() : ioLimitSW2.read();
}

bool Axis::readLimitSwitch() {
    return ucOriginDirection == Clockwise ? ioLimitSW2.read() : ioLimitSW1.read();
}

[[nodiscard]] float Axis::getPositionInMM() const {
    return xCurrentPosition / fStepsPerMM;
}

BaseType_t Axis::enqueueMove(Move const & message) {
    return xMoveQueue.push_back(message, portMAX_DELAY);
}

[[nodiscard]] Axis::Move Axis::dequeueMove() {
    return xMoveQueue.pop_front();
}

void Axis::onNewConfiguration(size_t uxSizeInMM, uint8_t ucOriginDirection) {
    this->uxSizeInMM = uxSizeInMM;
    if (this->ucOriginDirection != ucOriginDirection)
        xCurrentPosition = xMaximumPosition - xCurrentPosition;
    this->ucOriginDirection = ucOriginDirection;
    enqueueMove({ 0 - getPositionInMM(), Axis::kMaximumPPS });
}

void Axis::onMoveComplete() {
    stopStepping();

    portBASE_TYPE xHigherPriorityWoken = pdFALSE;
    xSemaphoreGiveFromISR(xMoveComplete, &xHigherPriorityWoken);
    portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void Axis::prvAxisTask(void* pvParameters) {
    Axis& axis = *reinterpret_cast<Axis*>(pvParameters);

    axis.calibrate();

    while (true) {
        Move const move = axis.dequeueMove();
        xEventGroupSync(xEventGroup, 1 << axis.uxTaskID, uxBitsToWaitFor & 0xFFF, portMAX_DELAY);
        axis.movef(move.fDistanceInMM, move.xStepsPerSecond);
    }
}
