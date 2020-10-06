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

static constexpr size_t kTicksPerSecond{ 1'000'000 }, kFrequency{ 50 }, kPeriod{ kTicksPerSecond / kFrequency - 1 };
constexpr static auto MalformedCode = "Malformed code\r\n", UnknownCode = "Unknown code\r\n", NotAGCode = "Not a GCode\r\n";
constexpr static auto OK = "OK\r\n";
static LPCPinMap constexpr pinmapXStep{ 0, 24 }, pinmapXDir{ 1,   0 }, pinmapXOrigin{ 0,  9 }, pinmapXLimit{ 0, 29 };
static LPCPinMap constexpr pinmapYStep{ 0, 27 }, pinmapYDir{ 0,  28 }, pinmapYOrigin{ 0,  0 }, pinmapYLimit{ 1,  3 };
static size_t width{ 150 }, height{ 100 };
static Axis* X, * Y;

struct MoveConfig {
	float x;
	float y;
	uint8_t isRelative;
	uint8_t plotterControl;
	void (*setPlotterControl)(uint8_t plotterControl) = nullptr;
};

static QueueWrapper<MoveConfig, 6>* xMoveQueue;

static void prvSetPenPosition(uint8_t penPosition) {
	LPC_SCT2->MATCHREL[2].U = kPeriod;
	LPC_SCT2->MATCHREL[3].U = kPeriod * penPosition / 255;
}

static void prvSetLaserPower(uint8_t laserPower) {
	LPC_SCT2->MATCHREL[4].U = kPeriod;
	LPC_SCT2->MATCHREL[5].U = kPeriod * laserPower / 255;
}

static void prvSetupHardware() {
	SystemCoreClockUpdate();
	Board_Init();
	heap_monitor_setup();
	ITM_init();
	auto const prescale = SystemCoreClock / kTicksPerSecond - 1;
	Chip_SCTPWM_Init(LPC_SCT2);
	LPC_SCT2->CONFIG = SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L;
	LPC_SCT2->CTRL_U = SCT_CTRL_PRE_L(prescale) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L;

	// SCTimer Events for X and Y Axes. Match condition and the state in which events occur is set by callback function.
	// No output is used, as the step pin is toggled manually in the interrupmove.
	LPC_SCT2->EVENT[0].CTRL = 0 << 0 | 1 << 12; // X axis event is set by Match 0 condition
	LPC_SCT2->EVENT[1].CTRL = 1 << 0 | 1 << 12; // Y axis event is set by Match 1 condition
	LPC_SCT2->RES = 0xF;
	LPC_SCT2->EVEN = SCT_EVT_0 | SCT_EVT_1;
	NVIC_SetPriority(SCT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(SCT2_IRQn);

	// SCTimer Events and output for pen servo. Match condition set by prvSetPenPosition function.
	LPC_SCT2->EVENT[2].STATE = LPC_SCT2->EVENT[3].STATE = 0x1;
	LPC_SCT2->EVENT[2].CTRL = 1 << 1 | 1 << 12;
	LPC_SCT2->EVENT[3].CTRL = 1 << 2 | 1 << 12;
	LPC_SCT2->OUT[0].SET = 1 << 2;
	LPC_SCT2->OUT[0].CLR = 1 << 3;
	Chip_SWM_MovablePortPinAssign(SWM_SCT2_OUT0_O, 0, 10);
	prvSetPenPosition(160);

	// SCTimer Events and output for laser. Match condition set by prvSetLaserPower function.
	LPC_SCT2->EVENT[4].STATE = LPC_SCT2->EVENT[5].STATE = 0x1;
	LPC_SCT2->EVENT[4].CTRL = 1 << 3 | 1 << 12;
	LPC_SCT2->EVENT[5].CTRL = 1 << 4 | 1 << 12;
	LPC_SCT2->OUT[1].SET = 1 << 4;
	LPC_SCT2->OUT[1].CLR = 1 << 5;
	Chip_SWM_MovablePortPinAssign(SWM_SCT2_OUT0_O, 0, 12);
	prvSetLaserPower(0);

	// Start timer.
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
		[](uint32_t stepsPerSecond) {
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
		[](uint32_t stepsPerSecond) {
			LPC_SCT2->MATCHREL[1].U = kTicksPerSecond / stepsPerSecond - 1;
			LPC_SCT2->EVENT[1].STATE = 1;
		},
		[]() {
			LPC_SCT2->EVENT[1].STATE = 0;
		}
	};

	xMoveQueue = new QueueWrapper<MoveConfig, 6>();

	xTaskCreate([](void* pvParameters){
		constexpr static Axis::Direction kTowardsOrigin{ Axis::Direction::Clockwise };
		uint8_t penUp{ 160 }, penDown{ 90 }, speed{ 80 };
		MoveConfig move;

		char buffer[RCV_BUFSIZE + 1];

		while (true) {
			USB_receive((uint8_t *) buffer, RCV_BUFSIZE);

			auto const letter = buffer[0];
			auto const number = std::atoi(buffer + 1);

			switch (letter) {
			case 'G': {
				switch (number) {
				case 1:
					if (std::sscanf(buffer + 3, "X%f Y%f A%hhu", &move.x, &move.y, &move.isRelative) == 3)
						xMoveQueue->push_back(move, portMAX_DELAY);
					else
						ITM_write(MalformedCode);
					break;

				case 28:
					move.x = move.y = 0;
					move.isRelative = false;
					xMoveQueue->push_back(move, portMAX_DELAY);
					break;

				default:
					ITM_write(UnknownCode);
					break;
				}
				break;
			}

			case 'M': {
				switch (number) {
				case 1:
					if (std::sscanf(buffer + 3, "%hhu", &move.plotterControl) == 1)
						move.setPlotterControl = prvSetPenPosition;
					else
						ITM_write(MalformedCode);
					break;

				case 2: {
					uint8_t tempUp{ 0 }, tempDown{ 0 };

					if (std::sscanf(buffer + 3, "U%hhu D%hhu", &tempUp, &tempDown) == 2) {
						penUp = tempUp;
						penDown = tempDown;
					}
					else
						ITM_write(MalformedCode);
					break;
				}

				case 4:
					if (std::sscanf(buffer + 3, "%hhu", &move.plotterControl) == 1)
						move.setPlotterControl = prvSetLaserPower;
					else
						ITM_write(MalformedCode);
					break;

				case 5: {
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

				case 10:
					sprintf(buffer, "M10 XY %d %d 0.00 0.00 A%d B%d S%d H0 U%d D%d\r\n", width, height, kTowardsOrigin, kTowardsOrigin, speed, penUp, penDown);
					USB_send((uint8_t*) buffer, strlen(buffer));
					break;

				case 11:
					USB_send((uint8_t*) "M11 0 0 0 0\r\n", 17);
					break;

				default:
					ITM_write(UnknownCode);
					break;
				}
				break;
			}

			default:
				ITM_write(NotAGCode);
				break;
			}
			USB_send((uint8_t *) OK, sizeof(OK));
		}
	}, "GCode Parser", configMINIMAL_STACK_SIZE + 256, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	xTaskCreate([](void* pvParameters) {
		while (true) {
			auto move = xMoveQueue->pop_front(portMAX_DELAY);

			if (move.setPlotterControl != nullptr)
				move.setPlotterControl(move.plotterControl);

			if (move.isRelative) {
				if (std::abs(move.x) < std::abs(move.y)) {
					X->enqueueMove({ Move::Relative, move.x, std::abs(move.x) * 1000 / std::abs(move.y) });
					Y->enqueueMove({ Move::Relative, move.y, 1000 });
				} else {
					X->enqueueMove({ Move::Relative, move.x, 1000 });
					Y->enqueueMove({ Move::Relative, move.y, std::abs(move.y) * 1000 / std::abs(move.x) });
				}
			} else {
				X->enqueueMove({ Move::Absolute, move.x, 1000 });
				Y->enqueueMove({ Move::Absolute, move.y, 1000 });
			}
		}
	}, "Plotter Co-ordinator", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	xTaskCreate(cdc_task, "CDC", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	vTaskStartScheduler();
}
