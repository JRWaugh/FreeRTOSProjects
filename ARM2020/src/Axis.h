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

using StepEnableCallback = void (*)(float);
using StepDisableCallback = void (*)();

class Axis {
public:
    enum Direction { Clockwise = 0, CounterClockwise };
    struct Move {
        enum { Absolute, Relative };
        uint8_t isRelative;
        float fDistanceInMM;
        float xStepsPerSecond;
    };

    static constexpr size_t kMaximumPPS{ 3000 };
    static constexpr EventBits_t uxEnableBit{ 1 << 7 };

    Axis(   size_t uxSizeInMM,
            uint8_t ucOriginDirection,
            DigitalIOPin&& ioStep,
            DigitalIOPin&& ioDirection,
            DigitalIOPin&& ioOriginSW,
            DigitalIOPin&& ioLimitSW,
            StepEnableCallback,
            StepDisableCallback
    );

    ~Axis();

    void move(bool isRelative, int32_t xStepsToMove, float fStepsPerSecond = kMaximumPPS);
    void movef(bool isRelative, float fDistanceToMove, float fStepsPerSecond = kMaximumPPS);
    void step();
    static void halt();
    static void resume();
    [[nodiscard]] bool readOriginSwitch();
    [[nodiscard]] bool readLimitSwitch();
    void onNewConfiguration(size_t uxSizeInMM, uint8_t ucOriginDirection) {
        this->uxSizeInMM = uxSizeInMM;
        if (this->ucOriginDirection != ucOriginDirection) {
            xCurrentPosition = xMaximumPosition - xCurrentPosition;
        }
        this->ucOriginDirection = ucOriginDirection;
        enqueueMove({ Axis::Move::Absolute, 0, Axis::kMaximumPPS });
    }
    BaseType_t enqueueMove(Move const & message);
    [[nodiscard]] Move dequeueMove();
    void calibrate();

private:
    static constexpr int32_t kPositionUnknown{ -1 };
    static void prvAxisTask(void* pvParameters);

    void onMoveComplete();

    size_t uxSizeInMM;
    uint8_t ucOriginDirection;
    DigitalIOPin ioStep, ioDirection, ioOriginSW, ioLimitSW;
    StepEnableCallback startStepping;
    StepDisableCallback stopStepping;
    std::atomic<int32_t> xStepsRemaining{ 0 }, xCurrentPosition{ 0 }, xMaximumPosition{ kPositionUnknown };
    float fStepsPerMM;

    SemaphoreHandle_t xMoveComplete{ xSemaphoreCreateBinary() };
    TaskHandle_t xTaskHandle;
    QueueWrapper<Move, 1> xMoveQueue;
};

#endif /* AXIS_H_ */
