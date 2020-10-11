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

static size_t constexpr kTicksPerSecond{ 1'000'000 }, kPenFrequency{ 50 }, kPenPeriod{ kTicksPerSecond / kPenFrequency };
static size_t uxPlotterWidth{ 150 }, uxPlotterHeight{ 100 };
static uint8_t ucPenUp{ 160 }, ucPenDown{ 90 }, ucSpeed{ 80 };
static Axis* X, * Y;

static void prvSetPenPosition(uint8_t ucPenPosition) {
    static constexpr size_t kMinDutyCycle{ kPenPeriod / 20 }, kMaxDutyCycle{ kPenPeriod / 10 }, kDelta{ kMaxDutyCycle - kMinDutyCycle };

    LPC_SCT0->MATCHREL[1].U = kMinDutyCycle + kDelta * ucPenPosition / 255;
    LPC_SCT0->OUT[0].SET = 1 << 0;
}

static void prvSetLaserPower(uint8_t ucLaserPower) {
    LPC_SCT0->MATCHREL[3].U = ucLaserPower;
    LPC_SCT0->OUT[1].SET = ucLaserPower > 0 ? 1 << 2 : 0; // Disable output if pulse width is 0
}

static void prvSetupHardware() {
    SystemCoreClockUpdate();
    Board_Init();
    heap_monitor_setup();
    ITM_init();

    Chip_SCT_Init(LPC_SCT0);
    Chip_SCT_Init(LPC_SCT2);
    LPC_SCT0->CONFIG = LPC_SCT2->CONFIG = SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L;
    LPC_SCT0->CTRL_U = LPC_SCT2->CTRL_U = SCT_CTRL_PRE_L(SystemCoreClock / kTicksPerSecond - 1) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L;

    // SCTimer config for pen servo motor.
    LPC_SCT0->MATCHREL[0].U = kPenPeriod - 1;
    LPC_SCT0->EVENT[0].STATE = LPC_SCT0->EVENT[1].STATE = 0x1;
    LPC_SCT0->EVENT[0].CTRL = 0 << 0 | 1 << 12;
    LPC_SCT0->EVENT[1].CTRL = 1 << 0 | 1 << 12;
    LPC_SCT0->OUT[0].CLR = 1 << 1;
    prvSetPenPosition(ucPenUp);
    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 10);

    // SCTimer config for laser.
    LPC_SCT0->MATCHREL[2].U = 256 - 1;
    LPC_SCT0->EVENT[2].STATE = LPC_SCT0->EVENT[3].STATE = 0x1;
    LPC_SCT0->EVENT[2].CTRL = 1 << 1 | 1 << 12;
    LPC_SCT0->EVENT[3].CTRL = 1 << 2 | 1 << 12;
    LPC_SCT0->OUT[1].CLR = 1 << 3;
    prvSetLaserPower(0);
    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT1_O, 0, 12);

    // SCTimer Events for X and Y Axes. Match condition and the state in which events occur is set by callback function.
    LPC_SCT2->EVENT[0].CTRL = 0 << 0 | 1 << 12; // X axis event is set by Match 0 condition
    LPC_SCT2->EVENT[1].CTRL = 1 << 0 | 1 << 12; // Y axis event is set by Match 1 condition
    LPC_SCT2->RES = 0xF;
    LPC_SCT2->EVEN = SCT_EVT_0 | SCT_EVT_1;
    NVIC_SetPriority(SCT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(SCT2_IRQn);

    // Start timers.
    LPC_SCT0->CTRL_L &= ~SCT_CTRL_HALT_L;
    LPC_SCT2->CTRL_L &= ~SCT_CTRL_HALT_L;
}

extern "C" {
void vConfigureTimerForRunTimeStats() {
    Chip_SCT_Init(LPC_SCT3);
    LPC_SCT3->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCT3->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L;
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
        uxPlotterWidth,
        { { 0, 24 }, false, false, false },
        { { 1,  0 }, false, false, false },
        { { 0,  9 }, true, true, true },
        { { 0, 29 }, true, true, true },
        [](float stepsPerSecond) {
            LPC_SCT2->MATCHREL[0].U = kTicksPerSecond / stepsPerSecond - 1;
            LPC_SCT2->EVENT[0].STATE = 1;
        },
        []() {
            LPC_SCT2->EVENT[0].STATE = 0;
        }
    };

    Y = new Axis{
        uxPlotterHeight,
        { { 0, 27 }, false, false, false },
        { { 0, 28 }, false, false, false },
        { { 0,  0 }, true, true, true },
        { { 1,  3 }, true, true, true },
        [](float stepsPerSecond) {
            LPC_SCT2->MATCHREL[1].U = kTicksPerSecond / stepsPerSecond - 1;
            LPC_SCT2->EVENT[1].STATE = 1;
        },
        []() {
            LPC_SCT2->EVENT[1].STATE = 0;
        }
    };

    DigitalIOPin* ioButtonResume = new DigitalIOPin{ { 0, 8 }, true, true, true, PIN_INT0_IRQn, [](bool pressed) {
        if (pressed) {
            if (X != nullptr)
                X->resume();

            if (Y != nullptr)
                Y->resume();
        }
    }};

    DigitalIOPin* ioButtonHalt = new DigitalIOPin{ { 1, 6 }, true, true, true, PIN_INT1_IRQn, [](bool pressed) {
        if (pressed) {
            if (X != nullptr)
                X->halt();

            if (Y != nullptr)
                Y->halt();
        }
    }};

    xTaskCreate([](void* pvParameters) {
        static auto constexpr MalformedCode = "Malformed code\r\n", UnknownCode = "Unknown code\r\n", OK = "OK\r\n";
        enum { G1 = 'G' + 1, G28 = 'G' + 28, M1 = 'M' + 1, M2 = 'M' + 2, M4 = 'M' + 4, M5 = 'M' + 5, M10 = 'M' + 10, M11 = 'M' + 11 };
        float fX, fY;
        uint8_t isRelative, ucPulseWidth;
        char pcBuffer[RCV_BUFSIZE];

        while (true) {
            USB_receive(pcBuffer, RCV_BUFSIZE);

            ITM_write(pcBuffer);

            auto const letter = pcBuffer[0];
            auto const number = std::atoi(pcBuffer + 1);

            switch (letter + number) {
            case G1: // Command from mDraw to move to X/Y co-ordinate
                if (std::sscanf(pcBuffer + 3, "X%f Y%f A%hhu", &fX, &fY, &isRelative) == 3) {
                    if (isRelative) {
                        if (std::abs(fX) < std::abs(fY)) {
                            X->enqueueMove({ Axis::Move::Relative, fX, std::abs(fX) * Axis::kMaximumPPS / std::abs(fY) });
                            Y->enqueueMove({ Axis::Move::Relative, fY, Axis::kMaximumPPS });
                        } else {
                            X->enqueueMove({ Axis::Move::Relative, fX, Axis::kMaximumPPS });
                            Y->enqueueMove({ Axis::Move::Relative, fY, std::abs(fY) * Axis::kMaximumPPS / std::abs(fX) });
                        }
                    } else {
                        X->enqueueMove({ Axis::Move::Absolute, fX, Axis::kMaximumPPS });
                        Y->enqueueMove({ Axis::Move::Absolute, fY, Axis::kMaximumPPS });
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
                if (std::sscanf(pcBuffer + 3, "%hhu", &ucPulseWidth) == 1) {
                    while (LPC_SCT2->EVENT[0].STATE || LPC_SCT2->EVENT[1].STATE); // Poll until the steppers have stopped and they set their STATE registers to 0.
                    prvSetPenPosition(ucPulseWidth);
                } else
                    ITM_write(MalformedCode);
                break;

            case M2: { // Command from mDraw to save pen up and pen down positions
                uint8_t tempUp{ 0 }, tempDown{ 0 };

                if (std::sscanf(pcBuffer + 3, "U%hhu D%hhu", &tempUp, &tempDown) == 2) {
                    ucPenUp = tempUp;
                    ucPenDown = tempDown;
                }
                else
                    ITM_write(MalformedCode);
                break;
            }

            case M4: // Command from mDraw to set laser power
                if (std::sscanf(pcBuffer + 3, "%hhu", &ucPulseWidth) == 1) {
                    while (LPC_SCT2->EVENT[0].STATE || LPC_SCT2->EVENT[1].STATE); // Poll until the steppers have stopped and they set their STATE registers to 0.
                    prvSetLaserPower(ucPulseWidth);
                    if (ucPulseWidth == 0)
                        vTaskDelay(200); // Simulator takes a bit to realise we've stopped the laser.
                } else
                    ITM_write(MalformedCode);
                break;

            case M5: {
                uint8_t ucDirX{ 0 }, ucDirY{ 0 }; // Discarding these values for now. It's not like we're saving any of these values to flash anyway.
                if (std::sscanf(pcBuffer + 3, "A%hhu B%hhu H%u W%u S%hhu", &ucDirX, &ucDirY, &uxPlotterHeight, &uxPlotterWidth, &ucSpeed) != 5)
                    ITM_write(MalformedCode);
                break;
            }

            case M10: // Query from mDraw for width, height, origin point, ucSpeed, pen up and pen down positions
                sprintf(pcBuffer,
                        "M10 XY %u %u 0.00 0.00 A%d B%d S%hhu H0 U%hhu D%hhu\r\n",
                        uxPlotterWidth, uxPlotterHeight, Axis::kTowardsOrigin, Axis::kTowardsOrigin, ucSpeed, ucPenUp, ucPenDown);
                USB_send(pcBuffer, strlen(pcBuffer));
                break;

            case M11: // Query from mDraw for limit switch states
                sprintf(pcBuffer,
                        "M11 %hhu %hhu %hhu %hhu",
                        !X->readOriginSwitch(), !X->readLimitSwitch(), !Y->readOriginSwitch(), Y->readLimitSwitch());
                USB_send(pcBuffer, strlen(pcBuffer));
                break;

            default:
                ITM_write(UnknownCode);
                break;
            }

            USB_send(OK, sizeof(OK));
        }
    }, "GCode Parser", configMINIMAL_STACK_SIZE + 270, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(cdc_task, "CDC", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    vTaskStartScheduler();

    delete ioButtonResume;
    delete ioButtonHalt;
}
