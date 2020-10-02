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
#include <cmath>
#include "QueueWrapper.h"

#define ACCELERATING 0

struct Move {
	enum { Absolute, Relative };
	bool isRelative;
	BaseType_t numberOfSteps;
};

class Axis {
public:
	enum Direction { Clockwise = 0, CounterClockwise };

	static constexpr size_t kInitialPPS{ 600 }, kPPSDelta{ 100 }, kHalfStepsPerRev{ 400 };

	Axis(	DigitalIOPin ioStep,
			DigitalIOPin ioDirection,
			DigitalIOPin ioOriginSW,
			DigitalIOPin ioLimitSW,
			void (*start)(uint32_t stepsPerSecond),
			void (*stop)()
	) : ioStep{ ioStep }, ioDirection{ ioDirection }, ioOriginSW{ ioOriginSW }, ioLimitSW{ ioLimitSW }, start{ start }, stop{ stop } {
		xTaskCreate(prvAxisTask, nullptr, configMINIMAL_STACK_SIZE, this, tskIDLE_PRIORITY + 1UL, nullptr);
	}

	void moveToRelativePosition(int32_t relativePosition) {
		if (relativePosition == 0)
			return;
		else if (relativePosition > 0)
			setDirection(CounterClockwise);
		else if (relativePosition < 0)
			setDirection(Clockwise);

		numberOfSteps = stepsRemaining = abs(relativePosition);

#if ACCELERATING
		stepsPerSecondAccel = (xMaximumPPS - kInitialPPS) / (xJourneyLength * 0.1f);
		if (fLinearPPSAccel > xMaximumPPS)
			fLinearPPSAccel = xMaximumPPS;
		else if (fLinearPPSAccel < 0)
			fLinearPPSAccel = 0;
#endif
		move();
	}

	void moveToAbsolutePosition(int32_t absolutePosition) {
		moveToRelativePosition(absolutePosition - currentPosition);
	}

	void step() {
		ioStep.write(true);

		--stepsRemaining;
		Direction direction = this->getDirection();

		ioStep.write(false);

		if (direction == Clockwise) {
			--currentPosition;
			if (ioOriginSW.read()) {
				if (maximumPosition == kPositionUnknown)
					currentPosition = 0;
				endMove();
			} else if (!stepsRemaining)
				endMove();
		} else if (direction == CounterClockwise) {
			++currentPosition;

			if (ioLimitSW.read()) {
				if (maximumPosition == kPositionUnknown)
					maximumPosition = currentPosition.load();
				endMove();
			} else if (!stepsRemaining)
				endMove();
		}
#if ACCELERATING
		else if (stepsRemaining >= numberOfSteps * 0.90f)
			start(stepsPerSecond += stepsPerSecondAccel);
		else if (stepsRemaining <= numberOfSteps * 0.10f)
			start(stepsPerSecond -= stepsPerSecondAccel);
#endif
	}

	bool obstructed() {
		return ioOriginSW.read() && ioLimitSW.read();
	}

	void enqueueMove(Move const & message) {
		queue.push_back(message);
	}

	Move dequeueMove() {
		return queue.pop_front();
	}

	void calibrate() {
		moveToRelativePosition(INT16_MIN);
		moveToRelativePosition(INT16_MAX);
#if ACCELERATING == 1
		BaseType_t toggle = -1;
		xEventGroupSetBits(xPlotterFlagGroup, CalibratingPPS);
		while (xEventGroupGetBits(xPlotterFlagGroup) & CalibratingPPS) {
			xMaximumPPS += kPPSDelta;
			prvMoveToRelativePosition(xPlotterWidth * toggle);
			toggle = 0 - toggle;

			if (xRemainingJourneyLength == 0 && !ioXOriginPin->read() && !ioXLimitPin->read())
				xEventGroupClearBits(xPlotterFlagGroup, CalibratingPPS);
		}

		// Reset default PPS. Rework this to be less bad at some point!
		xMaximumPPS -= kPPSDelta;

		// We've lost our position so we need to find it again.
		xEventGroupClearBits(xPlotterFlagGroup, PositionFound);
		prvMoveToRelativePosition(INT16_MIN);
#endif
	}

private:
	void move() noexcept {
#if ACCELERATING
		//stepsPerSecond = kInitialPPS;
#endif
		start(stepsPerSecond);
		stepping = true;

		xSemaphoreTake(moveComplete, portMAX_DELAY);
	}

	void endMove() {
		if (stepping) {
			stop();
			stepping = false;
			portBASE_TYPE xHigherPriorityWoken = pdFALSE;
			xSemaphoreGiveFromISR(moveComplete, &xHigherPriorityWoken);
			portEND_SWITCHING_ISR(xHigherPriorityWoken);
		}
	}

	void setDirection(Direction direction) noexcept {
		ioDirection.write(direction);
	}

	[[nodiscard]] Direction getDirection() noexcept {
		return static_cast<Direction>(ioDirection.read());
	}

	static void prvAxisTask(void* pvParameters) {
		vTaskDelay(10);

		Axis* axis = (Axis*) pvParameters;

		while (axis->obstructed()); // Can change this to an event at some point.

		axis->calibrate();

		while (true) {
			Move move = axis->dequeueMove();
			if (move.isRelative) {
				axis->moveToRelativePosition(move.numberOfSteps);
			} else
				axis->moveToAbsolutePosition(move.numberOfSteps);
		}
	}

	static constexpr int32_t kPositionUnknown{ -1 };
	DigitalIOPin ioStep, ioDirection, ioOriginSW, ioLimitSW;
	void (*start)(uint32_t stepsPerSecond);
	void (*stop)();
	std::atomic<int32_t> numberOfSteps{ 0 }, stepsRemaining{ 0 }, currentPosition{ 0 }, maximumPosition{ -1 };
	std::atomic<bool> stepping{ false };
	int32_t stepsPerSecond{ 1000 };
	SemaphoreHandle_t moveComplete{ xSemaphoreCreateBinary() };
	QueueWrapper<Move, 6> queue;
#if ACCELERATING
	float volatile stepsPerSecond, stepsPerSecondAccel;
#endif
};

#endif /* AXIS_H_ */
