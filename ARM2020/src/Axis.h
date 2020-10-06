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
#include "QueueWrapper.h"

#define ACCELERATING 0

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
    static constexpr size_t kMaximumPPS{ 2000 }, kPPSDelta{ 100 }, kHalfStepsPerRev{ 400 };

    Axis(   size_t xSize,
            DigitalIOPin ioStep,
            DigitalIOPin ioDirection,
            DigitalIOPin ioOriginSW,
            DigitalIOPin ioLimitSW,
            StepStarter_t,
            StepStopper_t
    );

    void startMove(bool bIsRelative, int32_t xStepsToMove, float fStepsPerSecond = kMaximumPPS);
    void step();
    [[nodiscard]] bool obstructed() const;
    void enqueueMove(Move const & message);
    [[nodiscard]] Move dequeueMove();
    float calibrateStepsPerMM();

private:
    static constexpr int32_t kPositionUnknown{ -1 };
    static constexpr Direction kTowardsLimit{ static_cast<Direction>(!kTowardsOrigin) };
    static void prvAxisTask(void* pvParameters);

    void endMove();
    void setDirection(Direction direction);
    [[nodiscard]] Direction getDirection() const;
    size_t const xSizeInMM;
    DigitalIOPin ioStep, ioDirection, ioOriginSW, ioLimitSW;
    StepStarter_t start;
    StepStopper_t stop;
    std::atomic<int32_t> xNumberOfSteps{ 0 }, xStepsRemaining{ 0 }, xCurrentPosition{ 0 }, xMaximumPosition{ kPositionUnknown };
    SemaphoreHandle_t xMoveComplete{ xSemaphoreCreateBinary() };
    QueueWrapper<Move, 1> xMoveQueue;
#if ACCELERATING
    float volatile stepsPerSecond, stepsPerSecondAccel;
#endif
};

#endif /* AXIS_H_ */
