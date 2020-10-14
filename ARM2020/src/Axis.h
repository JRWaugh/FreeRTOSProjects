/*
 * Axis.h
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#ifndef AXIS_H_
#define AXIS_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "DigitalIOPin.h"
#include "QueueWrapper.h"
#include <atomic>

using StepEnableCallback = void (*)(float);
using StepDisableCallback = void (*)();

class Axis {
public:
    Axis(   size_t uxSizeInMM,
            uint8_t ucOriginDirection,
            DigitalIOPin&& ioStep,
            DigitalIOPin&& ioDirection,
            DigitalIOPin&& ioLimitSW1,
            DigitalIOPin&& ioLimitSW2,
            StepEnableCallback,
            StepDisableCallback
    );

    ~Axis();

    static constexpr size_t kMaximumPPS{ 3000 };
    enum Direction { Clockwise = 0, CounterClockwise };
    struct Move {
        float fDistanceInMM;
        float xStepsPerSecond;
    };

    void move(int32_t xStepsToMove, float fStepsPerSecond = kMaximumPPS);
    void movef(float fDistanceToMove, float fStepsPerSecond = kMaximumPPS);
    void step();
    void calibrate();
    EventBits_t waitForCalibration(TickType_t xTicksToWait);
    static void halt();
    static void resume();
    [[nodiscard]] bool readOriginSwitch();
    [[nodiscard]] bool readLimitSwitch();
    [[nodiscard]] float getPositionInMM() const;
    BaseType_t enqueueMove(Move const & message);
    [[nodiscard]] Move dequeueMove();
    void onNewConfiguration(size_t uxSizeInMM, uint8_t ucOriginDirection);
    size_t const uxTaskID; // Could write a getter, but it's const so it's fine so long as nobody goes out of their way to change its value...

private:
    static constexpr int32_t kPositionUnknown{ -1 };
    static void prvAxisTask(void* pvParameters);

    /* The same event bits are being used for everything.
     * Bits 0 and 1 are used for synchronisation, bits 12 and 13 are used for signalling the axes are calibrated.
     * Bit 23 is used for enabling/disabling the axis, and is accessed with halt() and resume() functions.
     * Deleting an Axis will break this functionality. Creating too many axes will also break this functionality. Oh well! */
    static size_t uxAxesCreated;
    static EventBits_t uxBitsToWaitFor;

    void onMoveComplete();

    size_t uxSizeInMM;
    uint8_t ucOriginDirection;
    DigitalIOPin ioStep, ioDirection, ioLimitSW1, ioLimitSW2;
    StepEnableCallback startStepping;
    StepDisableCallback stopStepping;
    std::atomic<int32_t> xStepsRemaining{ 0 }, xCurrentPosition{ 0 }, xMaximumPosition{ kPositionUnknown };
    float fStepsPerMM;

    SemaphoreHandle_t xMoveComplete{ xSemaphoreCreateBinary() };
    TaskHandle_t xTaskHandle;
    QueueWrapper<Move, 1> xMoveQueue;
};

#endif /* AXIS_H_ */
