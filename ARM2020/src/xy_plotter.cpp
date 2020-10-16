/*
 * xy_plotter.cpp
 *
 *  Created on: 1 Oct 2020
 *      Author: Joshua
 */

#include <Axis.h>
#include "board.h"
#include "FreeRTOS.h"
#include "heap_lock_monitor.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "ITM_write.h"
#include "user_vcom.h"

#define SIMULATOR 1

static size_t constexpr kTicksPerSecond{ 1'000'000 }, kPenFrequency{ 50 }, kPenPeriod{ kTicksPerSecond / kPenFrequency };
static Axis* X, * Y;
static DigitalIOPin* ioButtonResume, *ioButtonHalt, *ioLimitSwitch1, *ioLimitSwitch2, *ioLimitSwitch3, *ioLimitSwitch4;

struct {
    enum { Clockwise, CounterClockwise };
    uint32_t uxHeight;
    uint32_t uxWidth;
    uint8_t ucOriginDirX;
    uint8_t ucOriginDirY;
    uint8_t ucSpeed; // Not being used.
    uint8_t ucPenUp;
    uint8_t ucPenDown;
    char ucHeader[3]; // Technically a footer, but I want to avoid padding!

    uint8_t save() {
        strcpy(ucHeader, "XY");
        return Chip_EEPROM_Write(0x100, (uint8_t*) this, sizeof(*this));
    }

    void load() {
        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_EEPROM);
        Chip_SYSCTL_PeriphReset(RESET_EEPROM);

        if (Chip_EEPROM_Read(0x100, (uint8_t*) this, sizeof(*this)) == IAP_CMD_SUCCESS) {
            if (strcmp(ucHeader, "XY") != 0) {
                // Set default configuration if header doesn't match.
                uxHeight = 380;
                uxWidth = 310;
                ucOriginDirX = Clockwise;
                ucOriginDirY = Clockwise;
                ucSpeed = 50;
                ucPenUp = 160;
                ucPenDown = 90;
            }
        }
    }
} static PlotterConfig;

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
    Board_LED_Set(0, false);
    heap_monitor_setup();
    ITM_init();
    PlotterConfig.load();

    // Initialise SCTimers. Setting prescale, clearing match count, and halting all three.
    Chip_SCT_Init(LPC_SCT0);
    Chip_SCT_Init(LPC_SCT1);
    Chip_SCT_Init(LPC_SCT2);
    LPC_SCT0->CONFIG = LPC_SCT1->CONFIG = LPC_SCT2->CONFIG = SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L;
    LPC_SCT0->CTRL_U = LPC_SCT1->CTRL_U = LPC_SCT2->CTRL_U = SCT_CTRL_PRE_L(SystemCoreClock / kTicksPerSecond - 1) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L;

    // SCTimer config for pen servo motor.
    LPC_SCT0->MATCHREL[0].U = kPenPeriod - 1;
    LPC_SCT0->EVENT[0].STATE = LPC_SCT0->EVENT[1].STATE = 0x1;
    LPC_SCT0->EVENT[0].CTRL = 0 << 0 | 1 << 12;
    LPC_SCT0->EVENT[1].CTRL = 1 << 0 | 1 << 12;
    LPC_SCT0->OUT[0].CLR = 1 << 1;
    prvSetPenPosition(PlotterConfig.ucPenUp);
    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 10);

    // SCTimer config for laser.
    LPC_SCT0->MATCHREL[2].U = 256 - 1;
    LPC_SCT0->EVENT[2].STATE = LPC_SCT0->EVENT[3].STATE = 0x1;
    LPC_SCT0->EVENT[2].CTRL = 2 << 0 | 1 << 12;
    LPC_SCT0->EVENT[3].CTRL = 3 << 0 | 1 << 12;
    LPC_SCT0->OUT[1].CLR = 1 << 3;
    prvSetLaserPower(0);
    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT1_O, 0, 12);

    // SCTimer config for X and Y axes.
    LPC_SCT1->EVENT[0].STATE = LPC_SCT2->EVENT[0].STATE = 0x1;
    LPC_SCT1->EVENT[0].CTRL = LPC_SCT2->EVENT[0].CTRL = 0 << 0 | 1 << 12;
    LPC_SCT1->RES = LPC_SCT2->RES = 0x3;
    LPC_SCT1->EVEN = LPC_SCT2->EVEN = SCT_EVT_0;
    NVIC_EnableIRQ(SCT1_IRQn);
    NVIC_EnableIRQ(SCT2_IRQn);
    NVIC_SetPriority(SCT1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SCT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    // Start SCT0 timer.
    LPC_SCT0->CTRL_L &= ~SCT_CTRL_HALT_L;
}

extern "C" {
void vConfigureTimerForRunTimeStats() {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L;
}

void SCT1_IRQHandler(void) {
    if (X != nullptr)
        X->step();
    LPC_SCT1->EVFLAG = SCT_EVT_0;
}

void SCT2_IRQHandler(void) {
    if (Y != nullptr)
        Y->step();
    LPC_SCT2->EVFLAG = SCT_EVT_0;
}
}

int main(void) {
    prvSetupHardware();

    ioButtonResume = new DigitalIOPin{ { 0,  8 }, true, true, true, PIN_INT0_IRQn, Axis::ResumeCallback };
    ioButtonHalt   = new DigitalIOPin{ { 1,  6 }, true, true, true, PIN_INT1_IRQn, Axis::HaltCallback };

    // Can be moved to static variables inside Axis if anybody wants to make a pretty interface for it.
    ioLimitSwitch1 = new DigitalIOPin{ { 0,  9 }, true, true, true, PIN_INT2_IRQn, Axis::LimitSWCallback };
    ioLimitSwitch2 = new DigitalIOPin{ { 0, 29 }, true, true, true, PIN_INT3_IRQn, Axis::LimitSWCallback };
    ioLimitSwitch3 = new DigitalIOPin{ { 0,  0 }, true, true, true, PIN_INT4_IRQn, Axis::LimitSWCallback };
    ioLimitSwitch4 = new DigitalIOPin{ { 1,  3 }, true, true, true, PIN_INT5_IRQn, Axis::LimitSWCallback };

    X = new Axis{
        PlotterConfig.ucOriginDirX,
        { { 0, 27 }, false, false, false }, // 0, 24
        { { 0, 28 }, false, false, false }, // 1,  0
        [](float stepsPerSecond) {
            LPC_SCT1->MATCHREL[0].U = kTicksPerSecond / stepsPerSecond - 1;
            LPC_SCT1->CTRL_L &= ~SCT_CTRL_HALT_L;
        },
        []() {
            LPC_SCT1->CTRL_L |= SCT_CTRL_HALT_L;
        }
    };

    Y = new Axis{
        PlotterConfig.ucOriginDirY,
        { { 0, 24 }, false, false, false }, // 0, 27
        { { 1,  0 }, false, false, false }, // 0, 28
        [](float stepsPerSecond) {
            LPC_SCT2->MATCHREL[0].U = kTicksPerSecond / stepsPerSecond - 1;
            LPC_SCT2->CTRL_L &= ~SCT_CTRL_HALT_L;
        },
        []() {
            LPC_SCT2->CTRL_L |= SCT_CTRL_HALT_L;
        }
    };

    xTaskCreate([](void* pvParameters) {
        static auto constexpr MalformedCode = "Malformed code\r\n", UnknownCode = "Unknown code\r\n", OK = "OK\r\n";
        enum { G1 = 'G' + 1, G28 = 'G' + 28, M1 = 'M' + 1, M2 = 'M' + 2, M4 = 'M' + 4, M5 = 'M' + 5, M10 = 'M' + 10, M11 = 'M' + 11 };
        struct {
            float fX{ 0 }, fY{ 0 }, fStepsPerMMX{ 0 }, fStepsPerMMY{ 0 };
            uint8_t isRelative{ 0 }, ucPulseWidth{ 0 };
            void (*setPulseWidth)(uint8_t ucPulseWidth){ nullptr };
        } move;

        char pcBuffer[RCV_BUFSIZE];

        X->waitForCalibration(portMAX_DELAY);
        Y->waitForCalibration(portMAX_DELAY);

        // Should probably allow for some wiggle room with the ratio, but it works fine with the simulator for now. This is a bit of a rush job.
        if (auto const fRatio = (float) X->getMaximumPosition() / Y->getMaximumPosition(); fRatio == (float) PlotterConfig.uxHeight / PlotterConfig.uxWidth) {
            Axis::swap(&X, &Y);
        } else if (fRatio != (float) PlotterConfig.uxWidth / PlotterConfig.uxHeight) {
            while (true) {
                Board_LED_Toggle(0);
                vTaskDelay(configTICK_RATE_HZ);
            }
        }

        move.fStepsPerMMX = X->getMaximumPosition() / PlotterConfig.uxWidth;
        move.fStepsPerMMY = Y->getMaximumPosition() / PlotterConfig.uxHeight;

        X->enqueueMove({ Move::Origin, Axis::kMaximumPPS });
        Y->enqueueMove({ Move::Origin, Axis::kMaximumPPS });

        while (true) {
            USB_receive(pcBuffer, RCV_BUFSIZE);
            ITM_write(pcBuffer);

            auto const letter = pcBuffer[0];
            auto const number = std::atoi(pcBuffer + 1);

            switch (letter + number) {
            case G1: // Command from mDraw to move to X/Y co-ordinate
                if (std::sscanf(pcBuffer + 3, "X%f Y%f A%hhu", &move.fX, &move.fY, &move.isRelative) == 3) {
                    X->waitForMoveEnd(portMAX_DELAY);
                    Y->waitForMoveEnd(portMAX_DELAY);

                    move.fX *= move.fStepsPerMMX;
                    move.fY *= move.fStepsPerMMY;
                    if (!move.isRelative) {
                        move.fX -= X->getCurrentPosition();
                        move.fY -= Y->getCurrentPosition();
                    }
                    int32_t const xStepsToMoveX = std::round(move.fX);
                    int32_t const xStepsToMoveY = std::round(move.fY);

                    if (move.setPulseWidth != nullptr) {
                        move.setPulseWidth(move.ucPulseWidth);
                        vTaskDelay(configTICK_RATE_HZ / 10);
#if SIMULATOR
                        // Additional time is needed to ensure the laser is turned off in the simulator
                        if (move.setPulseWidth == prvSetLaserPower)
                            vTaskDelay(configTICK_RATE_HZ / 10);
#endif
                        move.setPulseWidth = nullptr;
                    }

                    // Scale speed so that both axes will move a different number of steps in the same time
                    if (float fabsX{ std::abs(move.fX) }, fabsY{ std::abs(move.fY) }; fabsX < fabsY) {
                        X->enqueueMove({ xStepsToMoveX, fabsX * Axis::kMaximumPPS / fabsY });
                        Y->enqueueMove({ xStepsToMoveY, Axis::kMaximumPPS });
                    } else if (fabsX > fabsY) {
                        X->enqueueMove({ xStepsToMoveX, Axis::kMaximumPPS });
                        Y->enqueueMove({ xStepsToMoveY, fabsY * Axis::kMaximumPPS / fabsX });
                    } else {
                        X->enqueueMove({ xStepsToMoveX, Axis::kMaximumPPS });
                        Y->enqueueMove({ xStepsToMoveY, Axis::kMaximumPPS });
                    }
                }
                else
                    ITM_write(MalformedCode);
                break;

            case G28: // Command from mDraw to move to origin
                X->enqueueMove({ Move::Origin, Axis::kMaximumPPS });
                Y->enqueueMove({ Move::Origin, Axis::kMaximumPPS });
                break;

            case M1: // Command from mDraw to SET pen position
                if (std::sscanf(pcBuffer + 3, "%hhu", &move.ucPulseWidth) == 1)
                    move.setPulseWidth = prvSetPenPosition;
                else
                    ITM_write(MalformedCode);
                break;

            case M2: { // Command from mDraw to SAVE pen up and pen down positions
                uint8_t ucPenUp{ 0 }, ucPenDown{ 0 };

                if (std::sscanf(pcBuffer + 3, "U%hhu D%hhu", &ucPenUp, &ucPenDown) == 2) {
                    PlotterConfig.ucPenUp = ucPenUp;
                    PlotterConfig.ucPenDown = ucPenDown;
                    PlotterConfig.save();
                }
                else
                    ITM_write(MalformedCode);
                break;
            }

            case M4: // Command from mDraw to SET laser power
                if (std::sscanf(pcBuffer + 3, "%hhu", &move.ucPulseWidth) == 1)
                    move.setPulseWidth = prvSetLaserPower;
                else
                    ITM_write(MalformedCode);
                break;

            case M5: { // Command from mDraw to SAVE plotter configuration
                uint32_t uxPlotterHeight, uxPlotterWidth;
                uint8_t ucDirX, ucDirY, ucSpeed;

                if (std::sscanf(pcBuffer + 3, "A%hhu B%hhu H%lu W%lu S%hhu", &ucDirX, &ucDirY, &uxPlotterHeight, &uxPlotterWidth, &ucSpeed) == 5) {
                    PlotterConfig.ucOriginDirX = ucDirX;
                    PlotterConfig.ucOriginDirY = ucDirY;
                    PlotterConfig.uxHeight = uxPlotterHeight;
                    PlotterConfig.uxWidth = uxPlotterWidth;
                    PlotterConfig.ucSpeed = ucSpeed + 1;
                    PlotterConfig.save();

                    X->setOriginDirection(ucDirX);
                    Y->setOriginDirection(ucDirY);
                } else
                    ITM_write(MalformedCode);
                break;
            }

            case M10: // Query from mDraw for width, height, origin direction, speed, pen up and pen down positions
                sprintf(pcBuffer,
                        "M10 XY %lu %lu 0.00 0.00 A%hhu B%hhu S%hhu H0 U%hhu D%hhu\r\n",
                        PlotterConfig.uxWidth, PlotterConfig.uxHeight,
                        PlotterConfig.ucOriginDirX, PlotterConfig.ucOriginDirY, PlotterConfig.ucSpeed,
                        PlotterConfig.ucPenUp, PlotterConfig.ucPenDown);
                USB_send(pcBuffer, strlen(pcBuffer));
                break;

            case M11: // Query from mDraw for limit switch states
                sprintf(pcBuffer,
                        "M11 %hhu %hhu %hhu %hhu",
                        !ioLimitSwitch1->read(), !ioLimitSwitch2->read(), !ioLimitSwitch3->read(), !ioLimitSwitch4->read());
                USB_send(pcBuffer, strlen(pcBuffer));
                break;

            default:
                ITM_write(UnknownCode);
                break;
            }

            USB_send(OK, sizeof(OK));
        }
    }, "GCode Parser", configMINIMAL_STACK_SIZE + 280, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(cdc_task, "CDC", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    vTaskStartScheduler();

    delete ioButtonResume;
    delete ioButtonHalt;
}
