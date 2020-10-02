/*
 * plotter_main.cpp
 *
 *  Created on: 17 Sep 2020
 *      Author: Joshua
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "QueueWrapper.h"

#include "plotter_main.h"
#include "DigitalIOPin.h"
#include "ITM_write.h"

#include <cmath>
#include <atomic>

#define CALIBRATE_MAX_WIDTH 1
#define CALIBRATE_MAX_PPS 0
#define USING_SIM_PINS 1

struct Stepper {
	DigitalIOPin ioStep, ioDirection, ioOriginSW, ioLimitSW;
	void onStep() {
		ioStep.write(true);
		ioStep.write(false);
	}
};

namespace Plotter {

static constexpr LPCPinMap pinmapXStep{ 0, 24 }, pinmapXDir{ 1,  0 }, pinmapXOrigin{ 0,  9 }, pinmapXLimit{ 0, 29 };
static Stepper* stepperX, * stepperY;
static DigitalIOPin* ioXDirPin, * ioXOriginPin, *ioXLimitPin;
static void (*prvStartPulsingX)(BaseType_t xPulsesPerSecond);
static void (*prvStopPulsingX)();

static constexpr LPCPinMap pinmapYStep{ 0, 27 }, pinmapYDir{ 0,  28 }, pinmapYOrigin{ 1,  3 }, pinmapYLimit{ 0, 0 };
static DigitalIOPin* ioYDirPin, * ioYOriginPin, *ioYLimitPin;
static void (*prvStartPulsingY)(BaseType_t xPulsesPerSecond);
static void (*prvStopPulsingY)();

static constexpr BaseType_t kInitialPPS{ 600 }, kPPSDelta{ 100 }, kHalfStepsPerRev{ 400 };

static std::atomic<bool> bIsPlotting{ false };
static BaseType_t xMaximumPPS{ kInitialPPS }, xJourneyLength{ 0 };
static volatile float xCurrentPPS{ kInitialPPS }, fLinearPPSAccel{ 0 };

static std::atomic<BaseType_t > xCurrentPosition{ 0 }, xRemainingJourneyLength{ 0 }, xPlotterWidth{ 0 };
static EventGroupHandle_t xPlotterFlagGroup;
static SemaphoreHandle_t xMoveComplete;
static QueueWrapper<Message, 7>* xMessages;
static TickType_t xOriginTimeStamp{ 0 }, xLimitTimeStamp{ 0 };

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void inline prvSetDirection(Direction eDirection) noexcept {
	ioXDirPin->write(eDirection);
}

[[nodiscard]] static inline Direction prvGetDirection() noexcept {
	return static_cast<Direction>(ioXDirPin->read());
}

static inline Direction prvToggleDirection() noexcept {
	return static_cast<Direction>(ioXDirPin->toggle());
}

static void prvPrintStats() {
	static char buffer[200]{ 0 };
	sprintf(buffer,
			"Maximum PPS Value: %ld\r\n"
			"Maximum RPM Value: %ld\r\n"
			"Fastest time: %dms\r\n", xMaximumPPS, xMaximumPPS * 60 / kHalfStepsPerRev, abs(xOriginTimeStamp - xLimitTimeStamp));
	ITM_write(buffer);
}

static void prvMove() noexcept {
	xCurrentPPS = kInitialPPS;

	prvStartPulsingX(kInitialPPS);
	bIsPlotting = true;

	// Wait for semaphore given by prvTerminateMove
	xSemaphoreTake(xMoveComplete, portMAX_DELAY);

	// Let the switches stabilise. The motor is stopped at this point, so it's safe.
	vTaskDelay(1);
}

// Will be called when remaining steps reach 0 or when a limit switch is hit.
static void prvTerminateMove() {
	if (bIsPlotting) {
		prvStopPulsingX();
		bIsPlotting = false;

		portBASE_TYPE xHigherPriorityWoken = pdFALSE;
		xSemaphoreGiveFromISR(xMoveComplete, &xHigherPriorityWoken);
		portEND_SWITCHING_ISR(xHigherPriorityWoken);
	}
}

static void prvMoveToRelativePosition(BaseType_t xOffsetFromPosition) {
	if (xOffsetFromPosition == 0)
		return;
	else if (xOffsetFromPosition > 0)
		prvSetDirection(CounterClockwise);
	else if (xOffsetFromPosition < 0)
		prvSetDirection(Clockwise);

	xRemainingJourneyLength = xJourneyLength = abs(xOffsetFromPosition);

	fLinearPPSAccel = (xMaximumPPS - kInitialPPS) / (xJourneyLength * 0.1f);
	if (fLinearPPSAccel > xMaximumPPS)
		fLinearPPSAccel = xMaximumPPS;
	else if (fLinearPPSAccel < 0)
		fLinearPPSAccel = 0;

	prvMove();
}

static void prvMoveToAbsolutePosition(BaseType_t xAbsolutePosition) {
	prvMoveToRelativePosition(xAbsolutePosition - xCurrentPosition);
}

static void prvCalibratePlotter() {
	xEventGroupClearBits(xPlotterFlagGroup, PositionFound | MaxWidthFound);

	// Move to left so we can reset step counting, then move to right to find the total width
	prvMoveToRelativePosition(INT16_MIN);
	prvMoveToRelativePosition(INT16_MAX);

#if CALIBRATE_MAX_PPS == 1
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

	prvPrintStats();

	// We've lost our position so we need to find it again.
	xEventGroupClearBits(xPlotterFlagGroup, PositionFound);
	prvMoveToRelativePosition(INT16_MIN);
#endif
	// Return to centre
	prvToggleDirection();
	prvMoveToAbsolutePosition(xPlotterWidth / 2);
}

static void prvOriginSwitchHandler(void* pvParameters) {
	vTaskDelay(10);

	static SemaphoreHandle_t xOriginAlertSemaphore = xSemaphoreCreateBinary();
	ioXOriginPin->setOnIRQCallback([](bool pressed) {
		Board_LED_Set(0, pressed);
		if (pressed) {
			// We can't move any further, so terminate the move
			prvTerminateMove();

			// Notify the task so that it can update the state machine and light an LED
			portBASE_TYPE xHigherPriorityWoken = pdFALSE;
			xSemaphoreGiveFromISR(xOriginAlertSemaphore, &xHigherPriorityWoken);
			portEND_SWITCHING_ISR(xHigherPriorityWoken);
		}
	});

	while (true) {
		xSemaphoreTake(xOriginAlertSemaphore, portMAX_DELAY);
		vStop();

		auto const bits = xEventGroupGetBits(xPlotterFlagGroup) & PositionFound;
		if (bits != PositionFound) {
			xEventGroupSetBits(xPlotterFlagGroup, PositionFound);
			xCurrentPosition = 0;
		}

		xOriginTimeStamp = xTaskGetTickCount();
	}
}

static void prvLimitSwitchHandler(void* pvParameters) {
	vTaskDelay(10);

	static SemaphoreHandle_t xLimitAlertSemaphore = xSemaphoreCreateBinary();
	ioXLimitPin->setOnIRQCallback([](bool pressed) {
		Board_LED_Set(1, pressed);
		if (pressed) {
			// We can't move any further, so terminate the move
			prvTerminateMove();

			// Notify the task so that it can update the state machine and light an LED
			portBASE_TYPE xHigherPriorityWoken = pdFALSE;
			xSemaphoreGiveFromISR(xLimitAlertSemaphore, &xHigherPriorityWoken);
			portEND_SWITCHING_ISR(xHigherPriorityWoken);
		}
	});

	while (true) {
		xSemaphoreTake(xLimitAlertSemaphore, portMAX_DELAY);
		vStop();

		auto bits = xEventGroupGetBits(xPlotterFlagGroup) & (PositionFound | MaxWidthFound);
		if (bits == PositionFound) {
			xEventGroupSetBits(xPlotterFlagGroup, MaxWidthFound);
			xPlotterWidth = xCurrentPosition.load();
		} else if (bits == MaxWidthFound) {
			xEventGroupSetBits(xPlotterFlagGroup, PositionFound);
			xCurrentPosition = xPlotterWidth.load();
		}

		xLimitTimeStamp = xTaskGetTickCount();
	}
}

static void prvPlotterTask(void* pvParameters) {
	vTaskDelay(10);

	while (ioXOriginPin->read() && ioXLimitPin->read()); // Can change this to an event at some point.

#if CALIBRATE_MAX_WIDTH
	prvCalibratePlotter();
#endif

	while (true) {
		Message message = xMessages->pop_back();
		xEventGroupWaitBits(xPlotterFlagGroup, Flag::Go, pdFALSE, pdFALSE, portMAX_DELAY);
		switch (message.command) {
		case Message::MoveLeft:
			prvMoveToRelativePosition(-message.value);
			break;

		case Message::MoveRight:
			prvMoveToRelativePosition(message.value);
			break;

		case Message::SetPPS:
			xMaximumPPS = message.value;
			break;
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void vResume() {
	xEventGroupSetBits(xPlotterFlagGroup, Flag::Go);
}
void vStop() {
	 //////////////////////xEventGroupClearBits(xPlotterFlagGroup, Flag::Go);
}

BaseType_t xGetPlotterWidth() {
	xEventGroupWaitBits(xPlotterFlagGroup, Flag::MaxWidthFound, pdFALSE, pdFALSE, portMAX_DELAY);
	return xPlotterWidth;
}

void vOnXStep() noexcept {
	static DigitalIOPin ioXStepPin{ pinmapXStep, false, false, false };

	ioXStepPin.write(false);

	switch(prvGetDirection()) {
	case Clockwise:
		--xCurrentPosition;
		break;

	case CounterClockwise:
		++xCurrentPosition;
		break;
	}

	if (--xRemainingJourneyLength == 0) {
		prvTerminateMove();
	} else if (xRemainingJourneyLength >= xJourneyLength * 0.90f)
		prvStartPulsingX(xCurrentPPS += fLinearPPSAccel);
	else if (xRemainingJourneyLength <= xJourneyLength * 0.10f)
		prvStartPulsingX(xCurrentPPS -= fLinearPPSAccel);

	ioXStepPin.write(true); // Takes > 300 CPU cycles to handle this function, so that's a long enough delay for the stepper with our rinky-dink MCU!
}

void vOnYStep() noexcept {
	static DigitalIOPin ioYStepPin{ pinmapYStep, false, false, false };

	ioYStepPin.write(false);

	switch(prvGetDirection()) {
	case Clockwise:
		--xCurrentPosition;
		break;

	case CounterClockwise:
		++xCurrentPosition;
		break;
	}

	if (--xRemainingJourneyLength == 0) {
		prvTerminateMove();
	} else if (xRemainingJourneyLength >= xJourneyLength * 0.90f)
		prvStartPulsingX(xCurrentPPS += fLinearPPSAccel);
	else if (xRemainingJourneyLength <= xJourneyLength * 0.10f)
		prvStartPulsingX(xCurrentPPS -= fLinearPPSAccel);

	ioYStepPin.write(true); // Takes > 300 CPU cycles to handle this function, so that's a long enough delay for the stepper with our rinky-dink MCU!
}

BaseType_t xEnqueueMessage(Message::Command command, BaseType_t value) {
	xEventGroupWaitBits(xPlotterFlagGroup, Flag::Initialised, pdFALSE, pdFALSE, portMAX_DELAY);
	return xMessages->push_back({command, value});
}

BaseType_t xEnqueueMessage(Message const & message) {
	xEventGroupWaitBits(xPlotterFlagGroup, Flag::Initialised, pdFALSE, pdFALSE, portMAX_DELAY);
	return xMessages->push_back(message);
}

void vInit(void (*startPulsing)(BaseType_t xPulsesPerSecond), void (*stopPulsing)()) noexcept {
	if (xPlotterFlagGroup == nullptr)
		xPlotterFlagGroup = xEventGroupCreate();

	if (!(xEventGroupGetBits(xPlotterFlagGroup) & Initialised)) {
		ITM_init();
		prvStartPulsingX = startPulsing;
		prvStopPulsingX = stopPulsing;

		xMoveComplete = xSemaphoreCreateBinary();
		xMessages = new QueueWrapper<Message, 7>;

		ioXDirPin = new DigitalIOPin{ pinmapXDir, false, false, false };
		ioXOriginPin = new DigitalIOPin{ pinmapXOrigin, true, true, true, PIN_INT0_IRQn };
		ioXLimitPin = new DigitalIOPin{ pinmapXLimit, true, true, true, PIN_INT1_IRQn };

		ioYDirPin = new DigitalIOPin{ pinmapYDir, false, false, false };
		ioYOriginPin = new DigitalIOPin{ pinmapYOrigin, true, true, true, PIN_INT2_IRQn };
		ioYLimitPin = new DigitalIOPin{ pinmapYLimit, true, true, true, PIN_INT3_IRQn };

		xTaskCreate(prvOriginSwitchHandler, "OriginSWHandler", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);
		xTaskCreate(prvLimitSwitchHandler, "LimitSWHandler", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);
		xTaskCreate(prvPlotterTask, "Plotter Task", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

		xEventGroupSetBits(xPlotterFlagGroup, Initialised);
	}
}
}
