/*
 * Axis.cpp
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#include <Axis.h>

Axis::Axis(	size_t xSize,
		DigitalIOPin ioStep,
		DigitalIOPin ioDirection,
		DigitalIOPin ioOriginSW,
		DigitalIOPin ioLimitSW,
		void (*start)(uint32_t stepsPerSecond),
		void (*stop)()
) : xSize{ xSize }, ioStep{ ioStep }, ioDirection{ ioDirection }, ioOriginSW{ ioOriginSW }, ioLimitSW{ ioLimitSW }, start{ start }, stop{ stop } {
	ioStep.write(true);
	xTaskCreate(prvAxisTask, nullptr, configMINIMAL_STACK_SIZE, this, tskIDLE_PRIORITY + 1UL, nullptr);
}

void Axis::startMove(int32_t xStepsToMove, uint32_t xStepsPerSecond) noexcept {
	if (xStepsToMove == 0)
		return;
	else if (xStepsToMove > 0)
		setDirection(CounterClockwise);
	else if (xStepsToMove < 0)
		setDirection(Clockwise);

	numberOfSteps = stepsRemaining = abs(xStepsToMove);

#if ACCELERATING
	stepsPerSecondAccel = (xMaximumPPS - kInitialPPS) / (xJourneyLength * 0.1f);
	if (fLinearPPSAccel > xMaximumPPS)
		fLinearPPSAccel = xMaximumPPS;
	else if (fLinearPPSAccel < 0)
		fLinearPPSAccel = 0;
	stepsPerSecond = kInitialPPS;
#endif
	start(xStepsPerSecond);

	xSemaphoreTake(moveComplete, portMAX_DELAY);
}

void Axis::endMove() {
	stop();
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	xSemaphoreGiveFromISR(moveComplete, &xHigherPriorityWoken);
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void Axis::step() {
	ioStep.write(false);

	--stepsRemaining;
	Direction direction = this->getDirection();

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

	ioStep.write(true);
#if ACCELERATING
	else if (stepsRemaining >= numberOfSteps * 0.90f)
		start(stepsPerSecond += stepsPerSecondAccel);
	else if (stepsRemaining <= numberOfSteps * 0.10f)
		start(stepsPerSecond -= stepsPerSecondAccel);
#endif
}

bool Axis::obstructed() {
	return ioOriginSW.read() && ioLimitSW.read();
}

void Axis::enqueueMove(Move const & message) {
	queue.push_back(message, portMAX_DELAY);
}

Move Axis::dequeueMove() {
	return queue.pop_front();
}

void Axis::calibrate() {
	startMove(INT32_MIN);
	startMove(INT32_MAX);
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
	startMove(0 - currentPosition);

	fStepsPerMM = static_cast<float>(maximumPosition) / xSize;
}

[[nodiscard]] int32_t Axis::getMaxPosition() {
	return maximumPosition;
}

[[nodiscard]] int32_t Axis::getCurrentPosition() {
	return currentPosition;
}

[[nodiscard]] float Axis::getStepsPerMM() {
	return fStepsPerMM;
}

void Axis::setDirection(Direction direction) noexcept {
	ioDirection.write(direction);
}

[[nodiscard]] Axis::Direction Axis::getDirection() noexcept {
	return static_cast<Direction>(ioDirection.read());
}

void Axis::prvAxisTask(void* pvParameters) {
	static EventGroupHandle_t xEventGroup{ xEventGroupCreate() };
	static EventBits_t uxBitsToWaitFor{ 0 };
	static size_t xTasksCreated{ 0 };
	size_t xTaskID = xTasksCreated++;
	uxBitsToWaitFor = (uxBitsToWaitFor << 1) | 1;
	Axis* axis = (Axis*) pvParameters;

	vTaskDelay(10); // Let pins stabilise
	while (axis->obstructed());

	axis->calibrate();

	float const fStepsPerMM = axis->getStepsPerMM();

	while (true) {
		Move move = axis->dequeueMove();
		xEventGroupSync(xEventGroup, 1 << xTaskID, uxBitsToWaitFor, portMAX_DELAY);
		if (move.isRelative) {
			axis->startMove(move.fDistanceInMM * fStepsPerMM, move.xStepsPerSecond);
		} else
			axis->startMove(move.fDistanceInMM * fStepsPerMM - axis->getCurrentPosition(), move.xStepsPerSecond);
	}
}

