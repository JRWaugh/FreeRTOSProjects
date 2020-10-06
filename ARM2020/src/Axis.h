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
#include "event_groups.h"

#define ACCELERATING 0

struct Move {
	enum { Absolute, Relative };
	bool isRelative;
	float fDistanceInMM;
	size_t xStepsPerSecond;
};

class Axis {
public:
	enum Direction { Clockwise = 0, CounterClockwise };

	static constexpr size_t kInitialPPS{ 600 }, kPPSDelta{ 100 }, kHalfStepsPerRev{ 400 };

	Axis(	size_t xSize,
			DigitalIOPin ioStep,
			DigitalIOPin ioDirection,
			DigitalIOPin ioOriginSW,
			DigitalIOPin ioLimitSW,
			void (*start)(uint32_t stepsPerSecond),
			void (*stop)()
	);

	void startMove(int32_t xStepsToMove, uint32_t xStepsPerSecond = 1000) noexcept;
	void step();
	bool obstructed();
	void enqueueMove(Move const & message);
	Move dequeueMove();
	void calibrate();
	[[nodiscard]] int32_t getMaxPosition();
	[[nodiscard]] int32_t getCurrentPosition();
	[[nodiscard]] float getStepsPerMM();

private:
	static constexpr int32_t kPositionUnknown{ -1 };

	void endMove();
	void setDirection(Direction direction) noexcept;
	[[nodiscard]] Direction getDirection() noexcept;
	static void prvAxisTask(void* pvParameters);
	size_t xSize;
	float fStepsPerMM;
	DigitalIOPin ioStep, ioDirection, ioOriginSW, ioLimitSW;
	void (*start)(uint32_t stepsPerSecond);
	void (*stop)();
	std::atomic<int32_t> numberOfSteps{ 0 }, stepsRemaining{ 0 }, currentPosition{ 0 }, maximumPosition{ kPositionUnknown };
	int32_t stepsPerSecond{ 1000 };
	SemaphoreHandle_t moveComplete{ xSemaphoreCreateBinary() };
	QueueWrapper<Move, 1> queue;
#if ACCELERATING
	float volatile stepsPerSecond, stepsPerSecondAccel;
#endif
};

#endif /* AXIS_H_ */
