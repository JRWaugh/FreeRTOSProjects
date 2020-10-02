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

static constexpr size_t kTicksPerSecond{ 1'000'000 };
static LPCPinMap constexpr pinmapXStep{ 0, 24 }, pinmapXDir{ 1,   0 }, pinmapXOrigin{ 0,  9 }, pinmapXLimit{ 0, 29 };
static LPCPinMap constexpr pinmapYStep{ 0, 27 }, pinmapYDir{ 0,  28 }, pinmapYOrigin{ 0,  0 }, pinmapYLimit{ 1,  3 };
static Axis* X, * Y;

static void prvSetupHardware() {
	SystemCoreClockUpdate();
	Board_Init();
	heap_monitor_setup();
	auto const prescale = SystemCoreClock / kTicksPerSecond - 1;
	Chip_SCTPWM_Init(LPC_SCT2);
	LPC_SCT2->CONFIG = SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L;
	LPC_SCT2->CTRL_U = SCT_CTRL_PRE_L(prescale) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L;
	LPC_SCT2->CTRL_L |= 1 << 2;
	LPC_SCT2->EVENT[0].STATE = 1 << 0;
	LPC_SCT2->EVENT[1].STATE = 1 << 0;
	LPC_SCT2->EVENT[0].CTRL = 0 << 0 | 1 << 12;
	LPC_SCT2->EVENT[1].CTRL = 1 << 0 | 1 << 12;
	LPC_SCT2->RES = 3 << 0 | 3 << 2;
	LPC_SCT2->CTRL_L &= ~(1 << 2);
	NVIC_SetPriority(SCT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(SCT2_IRQn);
}

extern "C" {
void vConfigureTimerForRunTimeStats() {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L;
}

void SCT2_IRQHandler(void) {
	/* Note: I'm not disabling the timer at any point, instead I'm disabling different events causing an interrupt.
	 * For that reason, I have to check if a bit is set in EVEN as well as whether an EVFLAG bit been set because of a timer match.
	 */
	if (LPC_SCT2->EVEN & LPC_SCT2->EVFLAG & SCT_EVT_0 && X != nullptr)
		X->step();

	if (LPC_SCT2->EVEN & LPC_SCT2->EVFLAG & SCT_EVT_1 && Y != nullptr)
		Y->step();

	LPC_SCT2->EVFLAG = SCT_EVT_0 | SCT_EVT_1;
}
}

int main(void) {
	prvSetupHardware();

	X = new Axis {
		{ pinmapXStep, false, false, false },
		{ pinmapXDir, false, false, false },
		{ pinmapXOrigin, true, true, true },
		{ pinmapXLimit, true, true, true },
		[](uint32_t stepsPerSecond) {
			LPC_SCT2->MATCHREL[0].U = kTicksPerSecond / stepsPerSecond - 1;
			Chip_SCT_ClearEventFlag(LPC_SCT2, SCT_EVT_0);
			Chip_SCT_EnableEventInt(LPC_SCT2, SCT_EVT_0);
		},
		[]() {
			Chip_SCT_DisableEventInt(LPC_SCT2, SCT_EVT_0);
		}
	};

	Y = new Axis {
		{ pinmapYStep, false, false, false },
		{ pinmapYDir, false, false, false },
		{ pinmapYOrigin, true, true, true },
		{ pinmapYLimit, true, true, true },
		[](uint32_t stepsPerSecond) {
			LPC_SCT2->MATCHREL[1].U = kTicksPerSecond / stepsPerSecond - 1;
			Chip_SCT_ClearEventFlag(LPC_SCT2, SCT_EVT_1);
			Chip_SCT_EnableEventInt(LPC_SCT2, SCT_EVT_1);
		},
		[]() {
			Chip_SCT_DisableEventInt(LPC_SCT2, SCT_EVT_1);
		}
	};

	X->enqueueMove({ Axis::Message::Relative, -2000 });
	Y->enqueueMove({ Axis::Message::Relative, -2000 });

#if 0
	xTaskCreate([](){
		char g_code[256]{ 0 };
		while (true) {
			switch (g_code[0]) {
			case 'G': {
				switch (std::atoi(g_code + 1)) {
				case 1: {
					float x{ 0 }, y{ 0 };
					uint8_t relative{ 0 };

					if (std::sscanf(g_code + 3, "X%f Y%f A%c", &x, &y, &relative) == 3)
						plotter->onG1Received(x, y, relative);
					else
						plotter->onError(kMalformedCode);
					break;
				}

				case 28:
					plotter->onG28Received();
					break;

				default:
					plotter->onError(kUnknownCode);
					break;
				}
				break;
			}

			case 'M': {
				switch (std::atoi(g_code + 1)) {
				case 1: {
					uint8_t pen_position{ 0 };

					if (std::sscanf(g_code + 3, "%c", &pen_position) == 1)
						plotter->onM1Received(pen_position);
					else
						plotter->onError(kMalformedCode);
					break;
				}

				case 2: {
					uint8_t up{ 0 }, down{ 0 };

					if (std::sscanf(g_code + 3, "U%c D%c", &up, &down) == 2)
						plotter->onM2Received(up, down);
					else
						plotter->onError(kMalformedCode);
					break;
				}

				case 4: {
					uint8_t laser_power{ 0 };

					if (std::sscanf(g_code + 3, "%c", &laser_power) == 1)
						plotter->onM4Received(laser_power);
					else
						plotter->onError(kMalformedCode);

					break;
				}

				case 5: {
					uint8_t a_step{ 0 }, b_step{ 0 }, speed{ 0 };
					uint32_t height{ 0 }, width{ 0 };

					if (std::sscanf(g_code + 3, "A%c B%c H%ld W%ld S%c", &a_step, &b_step, &height, &width, &speed) == 5)
						plotter->onM5Received(a_step, b_step, height, width, speed);
					else
						plotter->onError(kMalformedCode);

					break;
				}

				case 10:
					plotter->onM10Received();
					break;

				case 11:
					plotter->onM11Received();
					break;

				default:
					plotter->onError(kUnknownCode);
					break;
				}
				break;
			}

			default:
				plotter->onError(kNotAGCode);
				break;
			}
		}
	}, "Code Parser", configMINIMAL_STACK_SIZE + 256, nullptr, tskIDLE_PRIORITY + 1UL, nullptr)
#endif

	vTaskStartScheduler();
}
