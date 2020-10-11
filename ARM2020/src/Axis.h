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
#include "DigitalIOPin.h"
#include <atomic>
#include "event_groups.h"
#include "QueueWrapper.h"

using StepStarter_t = void (*)(float);
using StepStopper_t = void (*)();

class Axis {
public:
    struct Move {
        enum { Absolute, Relative };
        bool isRelative;
        float fDistanceInMM;
        float xStepsPerSecond;
    };

    enum Direction { Clockwise = 0, CounterClockwise };

    static constexpr Direction kTowardsOrigin{ Clockwise };
    static constexpr size_t kMaximumPPS{ 2000 };

    Axis(   size_t xSizeInMM,
            DigitalIOPin&& ioStep,
            DigitalIOPin&& ioDirection,
            DigitalIOPin&& ioOriginSW,
            DigitalIOPin&& ioLimitSW,
            StepStarter_t,
            StepStopper_t
    );

    ~Axis();

    void move(bool isRelative, int32_t xStepsToMove, float fStepsPerSecond = kMaximumPPS);
    void step();
    void halt();
    void resume();
    [[nodiscard]] bool readOriginSwitch();
    [[nodiscard]] bool readLimitSwitch();
    BaseType_t enqueueMove(Move const & message);
    [[nodiscard]] Move dequeueMove();
    float calibrateStepsPerMM();

private:
    static constexpr int32_t kPositionUnknown{ -1 };
    static constexpr Direction kTowardsLimit{ static_cast<Direction>(!kTowardsOrigin) };
    static constexpr EventBits_t uxEnableBit = 1 << 0;
    static void prvAxisTask(void* pvParameters);

    void onMoveComplete();

    size_t const xSizeInMM;
    DigitalIOPin ioStep, ioDirection, ioOriginSW, ioLimitSW;
    StepStarter_t startStepping;
    StepStopper_t stopStepping;
    std::atomic<int32_t> xStepsRemaining{ 0 }, xCurrentPosition{ 0 }, xMaximumPosition{ kPositionUnknown };

    EventGroupHandle_t xEventGroup{ xEventGroupCreate() };
    SemaphoreHandle_t xMoveComplete{ xSemaphoreCreateBinary() };
    TaskHandle_t xTaskHandle;
    QueueWrapper<Move, 1> xMoveQueue;
};

#endif /* AXIS_H_ */
