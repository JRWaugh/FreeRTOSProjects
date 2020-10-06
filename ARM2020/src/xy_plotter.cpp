/*
 * xy_plotter.cpp
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#include "board.h"
#include "FreeRTOS.h"
#include "heap_lock_monitor.h"
#include "Axis.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "ITM_write.h"
#include "user_vcom.h"

static constexpr size_t kTicksPerSecond{ 1'000'000 }, kFrequency{ 50 }, kPeriod{ kTicksPerSecond / kFrequency };
constexpr static auto MalformedCode = "Malformed code\r\n", UnknownCode = "Unknown code\r\n", NotAGCode = "Not a GCode\r\n";
constexpr static auto OK = "OK\r\n";
static LPCPinMap constexpr pinmapXStep{ 0, 24 }, pinmapXDir{ 1,   0 }, pinmapXOrigin{ 0,  9 }, pinmapXLimit{ 0, 29 };
static LPCPinMap constexpr pinmapYStep{ 0, 27 }, pinmapYDir{ 0,  28 }, pinmapYOrigin{ 0,  0 }, pinmapYLimit{ 1,  3 };
static size_t width{ 150 }, height{ 100 };
static Axis* X, * Y;

static void prvSetPenPosition(uint8_t penPosition) {
	// Min == 5% duty cycle, Max == 10% duty cycle
	static constexpr size_t kMinDutyCycle{ kPeriod / 20 }, kMaxDutyCycle{ kMinDutyCycle * 2 }, kDelta{ kMaxDutyCycle - kMinDutyCycle };
	LPC_SCT0->MATCHREL[1].U = kMinDutyCycle + kDelta * penPosition / 255;
}

static void prvSetLaserPower(uint8_t laserPower) {
	if (laserPower > 0) {
		LPC_SCT0->MATCHREL[3].U = laserPower;
		LPC_SCT0->OUT[1].SET = 1 << 2;
	} else {
		LPC_SCT0->OUT[1].SET = 0;
	}
}

static void prvSetupHardware() {
	SystemCoreClockUpdate();
	Board_Init();
	heap_monitor_setup();
	ITM_init();

	auto const prescale = SystemCoreClock / kTicksPerSecond - 1;

	Chip_SCTPWM_Init(LPC_SCT0);
	Chip_SCTPWM_Init(LPC_SCT2);
	LPC_SCT0->CONFIG = LPC_SCT2->CONFIG = SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L;
	LPC_SCT0->CTRL_U = LPC_SCT2->CTRL_U =SCT_CTRL_PRE_L(prescale) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L;

	// SCTimer config for pen servo motor.
	LPC_SCT0->MATCHREL[0].U = kPeriod - 1;
	prvSetPenPosition(160);
	LPC_SCT0->EVENT[0].STATE = LPC_SCT0->EVENT[1].STATE = 0x1;
	LPC_SCT0->EVENT[0].CTRL = 0 << 0 | 1 << 12;
	LPC_SCT0->EVENT[1].CTRL = 1 << 0 | 1 << 12;
	LPC_SCT0->OUT[0].SET = 1 << 0;
	LPC_SCT0->OUT[0].CLR = 1 << 1;
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 10);

	// SCTimer config for laser.
	LPC_SCT0->MATCHREL[2].U = 255;
	prvSetLaserPower(0);
	LPC_SCT0->EVENT[2].STATE = LPC_SCT0->EVENT[3].STATE = 0x1;
	LPC_SCT0->EVENT[2].CTRL = 1 << 1 | 1 << 12;
	LPC_SCT0->EVENT[3].CTRL = 1 << 2 | 1 << 12;
	LPC_SCT0->OUT[1].CLR = 1 << 3;
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT1_O, 0, 12);

	// SCTimer Events for X and Y Axes. Match condition and the state in which events occur is set by callback function.
	// No output is used, as the step pin is toggled manually in the interrupt.
	LPC_SCT2->EVENT[0].CTRL = 0 << 0 | 1 << 12; // X axis event is set by Match 0 condition
	LPC_SCT2->EVENT[1].CTRL = 1 << 0 | 1 << 12; // Y axis event is set by Match 1 condition
	LPC_SCT2->RES = 0xF;
	LPC_SCT2->EVEN = SCT_EVT_0 | SCT_EVT_1;
	NVIC_SetPriority(SCT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(SCT2_IRQn);

	// Start timers.
	LPC_SCT0->CTRL_L &= ~(1 << 2);
	LPC_SCT2->CTRL_L &= ~(1 << 2);
}

extern "C" {
void vConfigureTimerForRunTimeStats() {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L;
}

void SCT2_IRQHandler(void) {
	if (LPC_SCT2->EVFLAG & SCT_EVT_0 && X != nullptr) {
		LPC_SCT2->EVFLAG = SCT_EVT_0;
		X->step();
	}

	if (LPC_SCT2->EVFLAG & SCT_EVT_1 && Y != nullptr) {
		LPC_SCT2->EVFLAG = SCT_EVT_1;
		Y->step();
	}
}
}

int main(void) {
	prvSetupHardware();

	X = new Axis{
		width,
		{ pinmapXStep, false, false, false },
		{ pinmapXDir, false, false, false },
		{ pinmapXOrigin, true, true, true },
		{ pinmapXLimit, true, true, true },
		[](float stepsPerSecond) {
			LPC_SCT2->MATCHREL[0].U = kTicksPerSecond / stepsPerSecond - 1;
			LPC_SCT2->EVENT[0].STATE = 1;
		},
		[]() {
			LPC_SCT2->EVENT[0].STATE = 0;
		}
	};

	Y = new Axis{
		height,
		{ pinmapYStep, false, false, false },
		{ pinmapYDir, false, false, false },
		{ pinmapYOrigin, true, true, true },
		{ pinmapYLimit, true, true, true },
		[](float stepsPerSecond) {
			LPC_SCT2->MATCHREL[1].U = kTicksPerSecond / stepsPerSecond - 1;
			LPC_SCT2->EVENT[1].STATE = 1;
		},
		[]() {
			LPC_SCT2->EVENT[1].STATE = 0;
		}
	};

	xTaskCreate([](void* pvParameters) {
		enum { G1 = 'G' + 1, G28 = 'G' + 28, M1 = 'M' + 1, M2 = 'M' + 2, M4 = 'M' + 4, M5 = 'M' + 5, M10 = 'M' + 10, M11 = 'M' + 11 };
		float x, y;
		uint8_t moveIsRelative;
		uint8_t toolPulseWidth;
		uint8_t penUp{ 160 }, penDown{ 90 }, speed{ 80 };
		char buffer[RCV_BUFSIZE + 1];

		while (true) {
			USB_receive((uint8_t *) buffer, RCV_BUFSIZE);

			auto const letter = buffer[0];
			auto const number = std::atoi(buffer + 1);

			switch (letter + number) {
			case G1: // Command from mDraw to move to X/Y co-ordinate
				if (std::sscanf(buffer + 3, "X%f Y%f A%hhu", &x, &y, &moveIsRelative) == 3) {
					if (moveIsRelative) {
						if (std::abs(x) < std::abs(y)) {
							X->enqueueMove({ Axis::Move::Relative, x, std::abs(x) * Axis::kMaximumPPS / std::abs(y) });
							Y->enqueueMove({ Axis::Move::Relative, y, Axis::kMaximumPPS });
						} else {
							X->enqueueMove({ Axis::Move::Relative, x, Axis::kMaximumPPS });
							Y->enqueueMove({ Axis::Move::Relative, y, std::abs(y) * Axis::kMaximumPPS / std::abs(x) });
						}
					} else {
						X->enqueueMove({ Axis::Move::Absolute, x, Axis::kMaximumPPS });
						Y->enqueueMove({ Axis::Move::Absolute, y, Axis::kMaximumPPS });
					}
				}
				else
					ITM_write(MalformedCode);
				break;

			case G28: // Command from mDraw to move to origin
				X->enqueueMove({ Axis::Move::Absolute, 0, Axis::kMaximumPPS });
				Y->enqueueMove({ Axis::Move::Absolute, 0, Axis::kMaximumPPS });
				break;

			case M1: // Command from mDraw to set pen position
				if (std::sscanf(buffer + 3, "%hhu", &toolPulseWidth) == 1) {
					while (LPC_SCT2->EVENT[0].STATE || LPC_SCT2->EVENT[1].STATE); // Poll until the steppers have stopped and they set their STATE registers to 0.
					prvSetPenPosition(toolPulseWidth);
				} else
					ITM_write(MalformedCode);
				break;


			case M2: { // Command from mDraw to save pen up and pen down positions
				uint8_t tempUp{ 0 }, tempDown{ 0 };

				if (std::sscanf(buffer + 3, "U%hhu D%hhu", &tempUp, &tempDown) == 2) {
					penUp = tempUp;
					penDown = tempDown;
				}
				else
					ITM_write(MalformedCode);
				break;
			}


			case M4: // Command from mDraw to set laser power
				if (std::sscanf(buffer + 3, "%hhu", &toolPulseWidth) == 1) {
					while (LPC_SCT2->EVENT[0].STATE || LPC_SCT2->EVENT[1].STATE); // Poll until the steppers have stopped and they set their STATE registers to 0.
					prvSetLaserPower(toolPulseWidth);
					if (!toolPulseWidth)
						vTaskDelay(200); // Simulator takes a bit to realise we've stopped the laser.
				} else
					ITM_write(MalformedCode);
				break;

			case M5: {
				uint8_t tempXDir{ 0 }, tempYDir{ 0 }, tempSpeed{ 0 };
				uint32_t tempHeight{ 0 }, tempWidth{ 0 };

				if (std::sscanf(buffer + 3, "A%c B%c H%ld W%ld S%hhu", &tempXDir, &tempYDir, &tempHeight, &tempWidth, &tempSpeed) == 5) {
#if WHOCARES
					x_direction = static_cast<Axis::Direction>(tempXDir);
					y_direction = static_cast<Axis::Direction>(tempYDir);
					height = tempHeight;
					width = tempWidth;
					speed = tempSpeed;
#endif
				}
				else
					ITM_write(MalformedCode);
				break;
			}

			case M10: // Query from mDraw for width, height, origin point, speed, pen up and pen down positions
				sprintf(buffer, "M10 XY %d %d 0.00 0.00 A%d B%d S%d H0 U%d D%d\r\n", width, height, Axis::kTowardsOrigin, Axis::kTowardsOrigin, speed, penUp, penDown);
				USB_send((uint8_t*) buffer, strlen(buffer));
				break;

			case M11: // Query from mDraw for limit switch states
				USB_send((uint8_t*) "M11 0 0 0 0\r\n", 17);
				break;

			default:
				ITM_write(UnknownCode);
				break;
			}
			USB_send((uint8_t *) OK, sizeof(OK));
		}
	}, "GCode Parser", configMINIMAL_STACK_SIZE + 270, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	xTaskCreate(cdc_task, "CDC", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	vTaskStartScheduler();
}
