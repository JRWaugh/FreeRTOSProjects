/*
 * Lab6.cpp
 *
 *  Created on: 15 Sep 2020
 *      Author: Joshua
 */

#include "board.h"
#include "FreeRTOS.h"
#include "heap_lock_monitor.h"
#include "semphr.h"
#include "DigitalIOPin.h"
#include "LpcUart.h"
#include <cstring>
#include "queue.h"
#include "Axis.h"

#define WITH_RIT 0
#define WITH_SCT 1

static size_t constexpr kTicksPerSecond{ 1'000'000 };
static Axis* X, * Y;

static void prvSetupHardware() {
    SystemCoreClockUpdate();
    Board_Init();
    heap_monitor_setup();
#if WITH_RIT == 1
    Chip_RIT_Init(LPC_RITIMER);
    Chip_RIT_Disable(LPC_RITIMER);
    Chip_RIT_EnableCompClear(LPC_RITIMER);
    NVIC_SetPriority(RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(RITIMER_IRQn);
#elif WITH_SCT == 1
    Chip_SCTPWM_Init(LPC_SCT2);
    LPC_SCT2->CONFIG = SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L;
    LPC_SCT2->CTRL_U = SCT_CTRL_PRE_L(SystemCoreClock / kTicksPerSecond - 1) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L;
    LPC_SCT2->EVENT[0].CTRL = 0 << 0 | 1 << 12;
    LPC_SCT2->RES = 3 << 0;
    LPC_SCT2->EVEN = 1 << 0;
    NVIC_SetPriority(SCT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(SCT2_IRQn);
#endif
}

extern "C" {
void vConfigureTimerForRunTimeStats() {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L;
}

#if WITH_RIT == 1
void RIT_IRQHandler(void) {
    Chip_RIT_ClearIntStatus(LPC_RITIMER);
    X->step();
}
#elif WITH_SCT == 1
void SCT2_IRQHandler(void) {
    if (LPC_SCT2->EVFLAG & SCT_EVT_0 && X != nullptr) {
        Chip_SCT_ClearEventFlag(LPC_SCT2, SCT_EVT_0);
        X->step();
    }
}
#endif
}

int main(void) {
    prvSetupHardware();

    xTaskCreate([](void* pvParameters) {
        LpcUart uart{ { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, { 0, 18 }, { 0, 13 } } };
        size_t value{ 0 };
        char buffer[60];
        float fStepsPerSecond{ Axis::kMaximumPPS };
        auto iter = std::begin(buffer);

        while (true) {
            uart.read(iter, 1, portMAX_DELAY);
            if (*iter == '\r' || *iter == '\n') {
                uart.write("\r\n");
                *iter = '\0';
                iter = std::begin(buffer);

                if (char* res = strstr(buffer, "go"); res != nullptr) {
                    // Plotter::vResume();
                } else if (res = strstr(buffer, "stop"); res != nullptr) {
                    // Plotter::vStop();
                } else if (res = strstr(buffer, "left "); res != nullptr) {
                    if (sscanf(res + 5, "%d", &value) == 1)
                        if (X->enqueueMove({ Axis::Move::Relative, -value, fStepsPerSecond }) == errQUEUE_FULL)
                            uart.write("Command queue full!\r\n");
                } else if (res = strstr(buffer, "right "); res != nullptr) {
                    if (sscanf(res + 6, "%d", &value) == 1)
                        if (X->enqueueMove({Axis::Move::Relative, value, fStepsPerSecond }) == errQUEUE_FULL)
                            uart.write("Command queue full!\r\n");
                }
                else if (res = strstr(buffer, "pps "); res != nullptr)
                    if (sscanf(res + 4, "%d", &value) == 1)
                       fStepsPerSecond = value;
            } else {
                uart.write(*iter);

                if (*iter == 127 && iter != std::begin(buffer))  // "Backspace", encoded as DEL by Putty
                    --iter;
                else if (++iter == std::end(buffer)) {
                    iter = std::begin(buffer);
                    uart.write("\r\nBuffer full!\r\n");
                }
            }
        }
    }, "Task 1", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    X = new Axis {
        0,
        { { 0, 24 }, false, false, false },
        { { 1,  0 }, false, false, false },
        { { 0,  9 }, true, true, true },
        { { 0, 29 }, true, true, true },
        [](float stepsPerSecond) {
#if WITH_RIT
            Chip_RIT_SetCounter(LPC_RITIMER, 0);
            Chip_RIT_Disable(LPC_RITIMER);
            Chip_RIT_SetCompareValue(LPC_RITIMER, SystemCoreClock / stepsPerSecond);
            Chip_RIT_Enable(LPC_RITIMER);
#elif WITH_SCT
            LPC_SCT2->MATCHREL[0].U = kTicksPerSecond / stepsPerSecond - 1;
            LPC_SCT2->EVENT[0].STATE = 1;
#endif
        },
        []() {
#if WITH_RIT
            Chip_RIT_Disable(LPC_RITIMER);
#elif WITH_SCT
            LPC_SCT2->EVENT[0].STATE = 0;
#endif
        }
    };
    vTaskStartScheduler();

    return 1;
}
