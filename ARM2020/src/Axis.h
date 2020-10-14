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

using MoveBeginCallback = void (*)(float);
using MoveEndCallback = void (*)();

class Axis {
public:
    Axis(   size_t uxSizeInMM,
            uint8_t ucOriginDirection,
            DigitalIOPin&& ioStep,
            DigitalIOPin&& ioDirection,
            DigitalIOPin&& ioLimitSW1,
            DigitalIOPin&& ioLimitSW2,
            MoveBeginCallback,
            MoveEndCallback
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
    EventBits_t waitForMoveEnd(TickType_t xTicksToWait);
    EventBits_t waitForCalibration(TickType_t xTicksToWait);
    static void halt();
    static void resume();
    [[nodiscard]] bool readOriginSwitch();
    [[nodiscard]] bool readLimitSwitch();
    [[nodiscard]] float getPositionInMM() const;
    BaseType_t enqueueMove(Move const & message);
    [[nodiscard]] Move dequeueMove();
    void onNewConfiguration(size_t uxSizeInMM, uint8_t ucOriginDirection);

    size_t const uxAxisID; // Could write a getter, but it's const so it's fine so long as nobody goes out of their way to change its value...

private:
    void moveEnd();

    enum { MoveReady = 0, Stopped = 8, Calibrated = 16, Enabled = 23 };
    static size_t uxAxesCreated;
    static EventGroupHandle_t xEventGroup;
    static EventBits_t uxBitsToWaitFor;

    static constexpr int32_t kPositionUnknown{ -1 };
    size_t uxSizeInMM;
    uint8_t ucOriginDirection;
    DigitalIOPin ioStep, ioDirection, ioLimitSW1, ioLimitSW2;
    MoveBeginCallback onMoveBegin;
    MoveEndCallback onMoveEnd;
    std::atomic<int32_t> xStepsRemaining{ 0 }, xCurrentPosition{ 0 }, xMaximumPosition{ kPositionUnknown };
    float fStepsPerMM;

    TaskHandle_t xTaskHandle;
    QueueWrapper<Move, 1> xMoveQueue;
    static void prvAxisTask(void* pvParameters);
};

#endif /* AXIS_H_ */
