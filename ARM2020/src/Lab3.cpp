#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "heap_lock_monitor.h"
#include "DigitalIOPin.h"
#include "ITM_write.h"
#include "LpcUart.h"
#include "QueueWrapper.h"

#define EX1 0
#define EX2 0
#define EX3 1

static LpcUart* uart;

extern "C" { void vConfigureTimerForRunTimeStats() {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}}

static void prvSetupHardware() {
    SystemCoreClockUpdate();
    Board_Init();
    heap_monitor_setup();
    ITM_init();
}

struct Event {
    char const * format;
    uint32_t data[3];
};

int main(void) {
    prvSetupHardware();

    uart = new LpcUart{ { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, { 0, 18 }, { 0, 13 } } };

#if EX1

    static constexpr int32_t kMessagePrintTotal{ -1 };
    static QueueWrapper<int32_t, 5>* xQueue = new QueueWrapper<int32_t, 5>();

    xTaskCreate([](void* pvParameters) {
        int32_t count{ 0 };
        char c;

        while (true) {
            uart->read(&c, 1, portMAX_DELAY);
            if (c == '\r' || c == '\n') {
                uart->write("\r\n");
                xQueue->push_back(count);
                count = 0;
            } else {
                uart->write(c);
                if (c == 127 && count > 0)// "Backspace", encoded as DEL by Putty
                    --count;
                else
                    ++count;
            }
        }
    }, "Task 1", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](void* pvParameters) {
        DigitalIOPin ioButton1{ { 0, 17 }, true, true, true, PIN_INT0_IRQn };

        while (true) {
            ioButton1.WFI(true, portMAX_DELAY);
            xQueue->push_back(kMessagePrintTotal);
        }
    }, "Task 2", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](void* pvParameters) {
        int32_t total{ 0 };

        while (true) {
            auto xNumberReceived = xQueue->pop_front(portMAX_DELAY);
            if (xNumberReceived == kMessagePrintTotal) {
                uart->print("You have typed %ld characters\r\n", total);
                total = 0;
            } else {
                total += xNumberReceived;
            }
        }
    }, "Task 3", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX2

    static QueueWrapper<int32_t, 20>* xQueue = new QueueWrapper<int32_t, 20>();
    static constexpr int32_t kMessageEmptyQueue{ 112 };

    xTaskCreate([](void* pvParameters) {
        while (true) {
            auto const xNumberGenerated = std::rand();
            xQueue->push_back(xNumberGenerated % kMessageEmptyQueue);
            vTaskDelay(xNumberGenerated % 400 + 100);
        }
    }, "Task 1", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](void* pvParameters) {
        DigitalIOPin ioButton1{ { 0, 17 }, true, true, true, PIN_INT0_IRQn };

        while (true) {
            ioButton1.WFI(true, portMAX_DELAY);
            xQueue->push_back(kMessageEmptyQueue);
        }
    }, "Task 2", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](void* pvParameters) {
        while (true) {
            auto xNumberReceived = xQueue->pop_front(portMAX_DELAY);
            uart->print("Number received: %ld\r\n", xNumberReceived);

            if (xNumberReceived == kMessageEmptyQueue) {
                uart->write("Help me!\r\n");
                vTaskDelay(800);
            }
        }
    }, "Task 3", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX3

    static QueueWrapper<Event, 5>* xQueue = new QueueWrapper<Event, 5>();

    xTaskCreate([](void* pvParameters) {
        while (true) {
            auto const event = xQueue->pop_front(portMAX_DELAY);
            ITM_print(event.format, event.data[0], event.data[1], event.data[2]);
        }
    }, "Task 1", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](void* pvParameters) {
        size_t count{ 0 };
        char c{ 0 };

        while (true) {
            uart->read(&c, 1, portMAX_DELAY);
            if (c == '\r' || c == '\n')
                uart->write("\r\n");
            else
                uart->write(c);

            if (std::isspace(c)) {
                if (count > 0)
                    xQueue->push_back({ "[Tick: %d] Word has %d letters\r\n", { xTaskGetTickCount(), count, 0 } });
                count = 0;
            } else if (c == 127 && count > 0) {
                --count;
            } else {
                ++count;
            }
        }
    }, "Task 2", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 2UL, nullptr);

    xTaskCreate([](void* pvParameters) {
        DigitalIOPin ioButton1{ { 0, 17 }, true, true, true, PIN_INT0_IRQn };

        while (true) {
            ioButton1.WFI(true, portMAX_DELAY);
            auto startTicks = xTaskGetTickCount();
            ioButton1.WFI(false, portMAX_DELAY);
            auto endTicks = xTaskGetTickCount();
            xQueue->push_back({ "[Tick: %d] Button held for %dms\r\n", endTicks, endTicks - startTicks, 0 });
        }
    }, "Task 3", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 2UL, nullptr);

#endif

    /* Start the scheduler */
    vTaskStartScheduler();

    /* Should never arrive here */
    return 1;
}

