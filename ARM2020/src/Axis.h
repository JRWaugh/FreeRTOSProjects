/*
 * Axis.h
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#ifndef AXIS_H_
#define AXIS_H_

#include "FreeRTOS.h"
#include "event_groups.h"
#include "DigitalIOPin.h"
#include "QueueWrapper.h"
#include <atomic>
#include <memory>




class Axis {
public:
    using MoveBeginCallback = void (*)(float);
    using MoveEndCallback = void (*)();
    struct Move {
        enum { Origin = INT32_MAX - 1 };
        int32_t xStepsToMove;
        float fStepsPerSecond;
    };
    static constexpr float kMaximumPPS{ 3000.0f };

    Axis(   uint8_t ucOriginDirection,
            std::unique_ptr<DigitalIOPin> ioStep,
            std::unique_ptr<DigitalIOPin> ioDirection,
            MoveBeginCallback,
            MoveEndCallback
    );

    ~Axis();

    void move(int32_t xStepsToMove, float fStepsPerSecond = kMaximumPPS);
    void endMove();
    void step();
    void calibrate();
    [[nodiscard]] int32_t getCurrentPosition() const;
    [[nodiscard]] int32_t getMaximumPosition() const;
    [[nodiscard]] size_t getID() const;
    EventBits_t waitForMoveEnd(TickType_t xTicksToWait) const;
    EventBits_t waitForCalibration(TickType_t xTicksToWait) const;
    BaseType_t enqueueMove(Move const & message);
    [[nodiscard]] Move dequeueMove();
    void setOriginDirection(uint8_t ucOriginDirection);
    static void swap(Axis* first, Axis* second) {
        std::swap(first->ioStep, second->ioStep);
        std::swap(first->ioDirection, second->ioDirection);

        int32_t xCurrentPosTemp = first->xCurrentPosition;
        int32_t xMaxPosTemp = first->xMaximumPosition;
        first->xCurrentPosition = second->xCurrentPosition.load();
        first->xMaximumPosition = second->xMaximumPosition.load();
        second->xCurrentPosition = xCurrentPosTemp;
        second->xMaximumPosition = xMaxPosTemp;
    }

private:
    enum { MoveReady = 0, Stopped = 8, Calibrated = 16, Enabled = 23 };
    static constexpr int32_t kPositionUnknown{ -1 };
    static Axis* Stepping;
    static size_t uxAxesCreated;
    size_t const uxID{ Axis::uxAxesCreated++ };

    uint8_t ucOriginDirection;
    std::unique_ptr<DigitalIOPin> ioStep, ioDirection;
    MoveBeginCallback onMoveBegin;
    MoveEndCallback onMoveEnd;
    std::atomic<bool> isStepping{ false };
    std::atomic<int32_t> xStepsRemaining{ 0 }, xCurrentPosition{ 0 }, xMaximumPosition{ kPositionUnknown };

    TaskHandle_t xTaskHandle;
    QueueWrapper<Move, 1> xMoveQueue;
    static EventGroupHandle_t xEventGroup;
    static void prvStepperTask(void* pvParameters);

public:
    static constexpr DigitalIOPin::onIRQCallback HaltCallback = [](bool pressed) {
        if (pressed)
            xEventGroupClearBitsFromISR(Axis::xEventGroup, 1UL << Axis::Enabled);
    };

    static constexpr DigitalIOPin::onIRQCallback ResumeCallback = [](bool pressed) {
        if (pressed) {
            portBASE_TYPE xHigherPriorityWoken = pdFALSE;
            xEventGroupSetBitsFromISR(Axis::xEventGroup, 1UL << Axis::Enabled, &xHigherPriorityWoken);
            portEND_SWITCHING_ISR(xHigherPriorityWoken);
        }
    };

    static constexpr DigitalIOPin::onIRQCallback LimitSWCallback = [](bool pressed) {
        if (pressed && Axis::Stepping) {
            Axis::Stepping->endMove();
            Axis::Stepping = nullptr;
        }
    };
};

#endif /* AXIS_H_ */
