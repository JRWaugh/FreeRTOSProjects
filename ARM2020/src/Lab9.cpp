/*
 * Lab9.cpp
 *
 *  Created on: 3 Oct 2020
 *      Author: Joshua
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "heap_lock_monitor.h"
#include "DigitalIOPin.h"
#include "LpcUart.h"
#include "event_groups.h"
#include <random>

#define EX1 1
#define EX2 0
#define EX3 0
#define EX3_2 1

static EventGroupHandle_t xEventGroup;
static LpcUart* uart;

static void prvSetupHardware() {
    SystemCoreClockUpdate();
    Board_Init();
    heap_monitor_setup();
    Board_LED_Set(0, false);
    Board_LED_Set(1, false);
    Board_LED_Set(2, false);
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
    xEventGroup = xEventGroupCreate();
    uart = new LpcUart{ { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, { 0, 18 }, { 0, 13 } } };
    DigitalIOPin* ioButton1 = new DigitalIOPin{{ 0, 17 }, true, true, true, PIN_INT0_IRQn };
#if EX2 | EX3 | EX3_2
    DigitalIOPin* ioButton2 = new DigitalIOPin{{ 1, 11 }, true, true, true, PIN_INT1_IRQn };
    DigitalIOPin* ioButton3 = new DigitalIOPin{{ 1,  9 }, true, true, true, PIN_INT2_IRQn };
#endif

#if EX1

    std::minstd_rand* std_rand = new std::minstd_rand{ std::random_device{}() };

    auto const prvEventWaitTask = [](void* pvParameters) {
        static size_t uxInitialTaskNumber{ 2 };

        size_t const uxTaskNumber{ uxInitialTaskNumber++ };
        TickType_t const xStartTicks{ xTaskGetTickCount() };
        auto& std_rand{ *reinterpret_cast<std::minstd_rand*>(pvParameters) };

        xEventGroupWaitBits(xEventGroup, 1 << 0, pdFALSE, pdFALSE, portMAX_DELAY);

        while (true) {
            uart->print("Task %d has been running for %d ticks\r\n", uxTaskNumber, xTaskGetTickCount() - xStartTicks);
            vTaskDelay(std_rand() % (configTICK_RATE_HZ + 1) + configTICK_RATE_HZ);
        }
    };

    xTaskCreate([](void* pvParameters) {
        auto& ioButton{ *reinterpret_cast<DigitalIOPin*>(pvParameters) };

        while (true) {
            ioButton.WFI(true, portMAX_DELAY);
            xEventGroupSetBits(xEventGroup, 1 << 0);
        }
    }, "Task 1", configMINIMAL_STACK_SIZE, ioButton1, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 2", configMINIMAL_STACK_SIZE + 80, std_rand, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 3", configMINIMAL_STACK_SIZE + 80, std_rand, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 4", configMINIMAL_STACK_SIZE + 80, std_rand, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX2

    auto const prvEventWaitTask = [](void* pvParameters) {
        static size_t xTasksCreated{ 1 };

        size_t const xTaskNumber{ xTasksCreated++ };
        TickType_t const xStartTime{ xTaskGetTickCount() };

        size_t count{ 0 };
        auto& ioButton{ *reinterpret_cast<DigitalIOPin*>(pvParameters) };

        while (true) {
            ioButton.WFI(true, portMAX_DELAY);
            if (++count == xTaskNumber) {
                xEventGroupSync(xEventGroup, 1 << xTaskNumber, 1 << 1 | 1 << 2 | 1 << 3, portMAX_DELAY);
                uart->print(
                        "Task Number: %d\r\n"
                        "Elapsed Ticks: %d\r\n"
                        "\r\n", xTaskNumber, xTaskGetTickCount() - xStartTime);
                vTaskSuspend(nullptr);
            }
        }
    };

    xTaskCreate(prvEventWaitTask, "Task 1", configMINIMAL_STACK_SIZE + 80, ioButton1, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 2", configMINIMAL_STACK_SIZE + 80, ioButton2, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 3", configMINIMAL_STACK_SIZE + 80, ioButton3, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX3

    auto const prvEventWaitTask = [](void* pvParameters) {
        static size_t xTasksCreated{ 0 };
        size_t const xTaskNumber{xTasksCreated++ };
        auto& ioButton{ *reinterpret_cast<DigitalIOPin*>(pvParameters) };

        while (true) {
            if (ioButton.WFI(false, portMAX_DELAY))
                xEventGroupSetBits(xEventGroup, 1 << xTaskNumber);
        }
    };

    xTaskCreate(prvEventWaitTask, "Task 1", configMINIMAL_STACK_SIZE, ioButton1, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 2", configMINIMAL_STACK_SIZE, ioButton2, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 3", configMINIMAL_STACK_SIZE, ioButton3, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](void* pvParameters){
        static UBaseType_t constexpr uxBitsToWaitFor{ 1 << 0 | 1 << 1 | 1 << 2 };
        TickType_t const xStartTicks{ xTaskGetTickCount() };
        TickType_t xPreviousTicks{ xStartTicks };
        while (true) {
            auto const uxBits = xEventGroupWaitBits(xEventGroup, uxBitsToWaitFor, pdTRUE, pdTRUE, configTICK_RATE_HZ * 30);

            TickType_t xCurrentTicks{ xTaskGetTickCount() };

            if (uxBits == uxBitsToWaitFor) {
                uart->print("OK!\r\nTime elapsed: %d\r\n", xCurrentTicks - xPreviousTicks);
                xPreviousTicks = xCurrentTicks;
            } else {
                for (size_t i = 0; i < 3; ++i)
                    if (!(uxBits & 1 << i))
                        uart->print("Task %d failed after %d ticks!\r\n", i, xCurrentTicks - xStartTicks);
                vTaskSuspend(nullptr);
            }
        }
    }, "Task 4", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX3_2

    auto const prvEventWaitTask = [](void* pvParameters) {
        static size_t xTasksCreated{ 0 };
        size_t const xTaskNumber{ xTasksCreated++ };
        auto& ioButton{ *reinterpret_cast<DigitalIOPin*>(pvParameters) };

        while (true) {
            if (ioButton.WFI(false, configTICK_RATE_HZ * 30) == pdFALSE) // Wait 30 seconds for falling edge
                xEventGroupSetBits(xEventGroup, 1 << xTaskNumber);
        }
    };

    xTaskCreate(prvEventWaitTask, "Task 1", configMINIMAL_STACK_SIZE, ioButton1, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 2", configMINIMAL_STACK_SIZE, ioButton2, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate(prvEventWaitTask, "Task 3", configMINIMAL_STACK_SIZE, ioButton3, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](void* pvParameters){
        UBaseType_t const uxBitsToWaitFor{ 1 << 0 | 1 << 1 | 1 << 2 };
        TickType_t const xStartTicks{ xTaskGetTickCount() };
        while (true) {
            xEventGroupWaitBits(xEventGroup, uxBitsToWaitFor, pdFALSE, pdFALSE, portMAX_DELAY);

            auto const uxBits = xEventGroupGetBits(xEventGroup); // Bit wonky, but means we'll be able to get multiple bits instead of just the first
            for (size_t i = 0; i < 3; ++i)
                if (uxBits & 1 << i)
                    uart->print("Task %d failed after %d ticks!\r\n", i, xTaskGetTickCount() - xStartTicks);
            vTaskSuspend(nullptr);
        }
    }, "Task 4", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#endif

    vTaskStartScheduler();
}
