/*
 * Lab7.cpp
 *
 *  Created on: 24 Sep 2020
 *      Author: Joshua
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user_vcom.h"
#include "heap_lock_monitor.h"
#include "DigitalIOPin.h"
#include <atomic>
#include <cstring>
#include "ITM_write.h"

#define EX1 0
#define EX2 1
#define EX3 1

static constexpr size_t kTicksPerSecond{ 1'000'000 };

#if EX1

static constexpr size_t kFrequency{ 1000 }, kPeriod{ kTicksPerSecond / kFrequency - 1 };
static constexpr int kMinDutyCycle{ 0 }, kMaxDutyCycle{ 100 };
std::atomic<int> xDutyCycle{ 5 };

#elif EX2

static constexpr size_t kFrequency{ 50 }, kPeriod{ kTicksPerSecond / kFrequency - 1 };
static constexpr float kMinDutyCycle{ 5.0f }, kMaxDutyCycle{ 10.0f }, kMeanDutyCycle{ (kMaxDutyCycle + kMinDutyCycle) / 2 };
std::atomic<float> xDutyCycle{ 7.5f }, kDelta{ 0.1f }; // Could avoid working with floats if I worked with pulse widths instead of duty cycles.

#elif EX3

static constexpr size_t kFrequency{ 1000 };
static constexpr size_t kPeriod{ 256 - 1 };
size_t xRedDutyCycle{ kPeriod / 2 }, xGreenDutyCycle{ kPeriod / 2 }, xBlueDutyCycle{ kPeriod / 2 };

#endif

static void prvSetupHardware() {
    SystemCoreClockUpdate();
    Board_Init();
    heap_monitor_setup();
    ITM_init();
    Board_LED_Set(0, false);
    Board_LED_Set(1, false);
    Board_LED_Set(2, false);
    auto const prescale = SystemCoreClock / kTicksPerSecond - 1;
    Chip_SCTPWM_Init(LPC_SCT0);

#if EX1 | EX2

    LPC_SCT0->CONFIG = SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L;
    LPC_SCT0->CTRL_U = SCT_CTRL_PRE_L(prescale) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L;
    LPC_SCT0->MATCHREL[0].U = kPeriod;
    LPC_SCT0->MATCHREL[1].U = kPeriod * xDutyCycle / 100;
    LPC_SCT0->EVENT[0].STATE = LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF;
    LPC_SCT0->EVENT[0].CTRL = 0 << 0 | 1 << 12;
    LPC_SCT0->EVENT[1].CTRL = 1 << 0 | 1 << 12;

#elif EX3
    LPC_SCT0->CONFIG = SCT_CONFIG_16BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L | SCT_CONFIG_AUTOLIMIT_H;
    LPC_SCT0->CTRL_U =
            SCT_CTRL_PRE_L(prescale) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L |
            SCT_CTRL_PRE_H(prescale) | SCT_CTRL_CLRCTR_H | SCT_CTRL_HALT_H;
    LPC_SCT0->MATCHREL[0].L = kPeriod;
    LPC_SCT0->MATCHREL[1].L = xRedDutyCycle;
    LPC_SCT0->MATCHREL[0].H = kPeriod;
    LPC_SCT0->MATCHREL[1].H = xGreenDutyCycle;
    LPC_SCT0->EVENT[0].STATE = LPC_SCT0->EVENT[1].STATE = LPC_SCT0->EVENT[2].STATE = LPC_SCT0->EVENT[3].STATE = 0xFFFFFFFF;
    LPC_SCT0->EVENT[0].CTRL = 0 << 0 | 0 << 4 | 1 << 12; // Related to match 0 | Event 0 belongs to high timer | Match condition only
    LPC_SCT0->EVENT[1].CTRL = 1 << 0 | 0 << 4 | 1 << 12;
    LPC_SCT0->EVENT[2].CTRL = 0 << 0 | 1 << 4 | 1 << 12; // Related to match 0 | Event 2 belongs to low timer  | Match condition only
    LPC_SCT0->EVENT[3].CTRL = 1 << 0 | 1 << 4 | 1 << 12;
    LPC_SCT0->CTRL_H &= ~(1 << 2); // Start high timer

    Chip_SCTPWM_Init(LPC_SCT1);
    LPC_SCT1->CONFIG = SCT_CONFIG_16BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L | SCT_CONFIG_AUTOLIMIT_H;
    LPC_SCT1->CTRL_U = SCT_CTRL_PRE_L(prescale) | SCT_CTRL_CLRCTR_L | SCT_CTRL_HALT_L;
    LPC_SCT1->MATCHREL[0].L = kPeriod;
    LPC_SCT1->MATCHREL[1].L = xBlueDutyCycle;
    LPC_SCT1->EVENT[0].STATE = LPC_SCT1->EVENT[1].STATE = 0xFFFFFFFF;
    LPC_SCT1->EVENT[0].CTRL = 0 << 0 | 0 << 4 | 1 << 12;
    LPC_SCT1->EVENT[1].CTRL = 1 << 0 | 0 << 4 | 1 << 12;
    LPC_SCT1->CTRL_L &= ~(1 << 2);

#endif

    LPC_SCT0->CTRL_L &= ~(1 << 2); // Start low timer, or unified timer if unified
}

extern "C" {
void vConfigureTimerForRunTimeStats() {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L;
}
}

int main(void) {
    prvSetupHardware();

#if EX1

    // Green LED
    // Note: The LED has inverted logic and "setting" it will turn it off.
    LPC_SCT0->OUT[0].SET = 1 << 1;
    LPC_SCT0->OUT[0].CLR = 1 << 0;
    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 3);

    xTaskCreate([](void* pvParameters) {
        static volatile TickType_t xTimerFrequency{ 10 };
        DigitalIOPin ioButton1{{ 0, 17 }, true, true, true };

        DigitalIOPin ioButton2{{ 1, 11 }, true, true, true, PIN_INT0_IRQn, [](bool pressed) {
            if (pressed)
                xTimerFrequency = 100;
            else
                xTimerFrequency = 10;
        }};

        DigitalIOPin ioButton3{{ 1,  9 }, true, true, true };

        while (true) {
            if (ioButton1.read()) {
                xDutyCycle += 1;
                if (xDutyCycle > kMaxDutyCycle)
                    xDutyCycle = kMaxDutyCycle;
                LPC_SCT0->MATCHREL[1].U = kPeriod * xDutyCycle / 100;
                ITM_print("Duty cycle: %d%%\r\n", xDutyCycle.load());
            } else if (ioButton3.read()) {
                xDutyCycle -= 1;
                if (xDutyCycle < kMinDutyCycle)
                    xDutyCycle = kMinDutyCycle;
                LPC_SCT0->MATCHREL[1].U = kPeriod * xDutyCycle / 100;
                ITM_print("Duty cycle: %d%%\r\n", xDutyCycle.load());
            }

            vTaskDelay(configTICK_RATE_HZ / xTimerFrequency);
        }
    }, "LED PWM", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX2

    // Servo
    LPC_SCT0->OUT[0].SET = 1 << 0;
    LPC_SCT0->OUT[0].CLR = 1 << 1;
    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 10);

    xTaskCreate([](void* pvParameters) {
        DigitalIOPin ioButton1{{ 0, 8 }, true, true, true };
        DigitalIOPin ioButton2{{ 1, 6 }, true, true, true };
        DigitalIOPin ioButton3{{ 1,  8 }, true, true, true };

        while (true) {
            if (ioButton1.read()) {
                xDutyCycle = xDutyCycle.load() - kDelta;
                if (xDutyCycle < kMinDutyCycle)
                    xDutyCycle = kMinDutyCycle;
            } else if (ioButton2.read()) {
                xDutyCycle = kMeanDutyCycle;
            } else if (ioButton3.read()) {
                xDutyCycle = xDutyCycle.load() + kDelta;
                if (xDutyCycle > kMaxDutyCycle)
                    xDutyCycle = kMaxDutyCycle;
            }
            LPC_SCT0->MATCHREL[1].U = kPeriod * xDutyCycle / 100;

            vTaskDelay(configTICK_RATE_HZ / 10);
        }
    }, "Servo PWM", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX3

    // Red LED
    LPC_SCT0->OUT[0].SET = 1 << 1; // Output 0 set by Event 1
    LPC_SCT0->OUT[0].CLR = 1 << 0; // Output 0 cleared by Event 0
    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O, 0, 25);

    // Green LED
    LPC_SCT0->OUT[1].SET = 1 << 3;
    LPC_SCT0->OUT[1].CLR = 1 << 2;
    Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT1_O, 0,  3);

    // Blue LED
    LPC_SCT1->OUT[0].SET = 1 << 1;
    LPC_SCT1->OUT[0].CLR = 1 << 0;
    Chip_SWM_MovablePortPinAssign(SWM_SCT1_OUT0_O, 1,  1);

    xTaskCreate([](void* pvParameters) {
        char rcv_buffer[RCV_BUFSIZE + 1];
        char cmd_buffer[RCV_BUFSIZE + 1];
        size_t count{ 0 };

        vTaskDelay(100); // wait until USB CDC semaphores are created

        while (true) {
            auto len = USB_receive((uint8_t *) rcv_buffer, RCV_BUFSIZE);

            // Copy received bytes into buffer and process chars at the same time
            for (size_t i = 0; i < len; ++i) {
                if (rcv_buffer[i] == '\r' || rcv_buffer[i] == '\n') {
                    USB_send((uint8_t *) "\r\n", 2);
                    cmd_buffer[count] = '\0';

                    if (char* res = strstr(cmd_buffer, "rgb #"); res != nullptr) {
                        if (sscanf(res + 5, "%02X%02X%02X", &xRedDutyCycle, &xGreenDutyCycle, &xBlueDutyCycle) == 3) {
                            LPC_SCT0->MATCHREL[1].L = xRedDutyCycle;
                            LPC_SCT0->MATCHREL[1].H = xGreenDutyCycle;
                            LPC_SCT1->MATCHREL[1].L = xBlueDutyCycle;
                        }
                    }
                    count = 0;
                } else {
                    USB_send((uint8_t *) &rcv_buffer[i], 1);

                    if (rcv_buffer[i] == 127 && count > 0)  // "Backspace", encoded as DEL by Putty
                        --count;
                    else {
                        cmd_buffer[count++] = rcv_buffer[i];
                        if (count == RCV_BUFSIZE) {
                            constexpr char error[] = "\r\nBuffer full!\r\n";
                            USB_send((uint8_t *) error, strlen(error));
                            count = 0;
                        }
                    }
                }
            }
        }
    }, "Command Reader", configMINIMAL_STACK_SIZE + 256, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(cdc_task, "CDC", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#endif
    vTaskStartScheduler();
}
