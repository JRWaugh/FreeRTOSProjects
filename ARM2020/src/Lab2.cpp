#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "heap_lock_monitor.h"

#include <array>
#include <random>
#include <algorithm>
#include "DigitalIOPin.h"
#include "LpcUart.h"

#define EX1 0
#define EX2 1
#define EX3 0

static LpcUart* uart;

/* Sets up system hardware */
static void prvSetupHardware() {
    SystemCoreClockUpdate();
    Board_Init();
    heap_monitor_setup();
}

/* the following is required if runtime statistics are to be collected */
extern "C" { void vConfigureTimerForRunTimeStats() {
    Chip_SCT_Init(LPC_SCTSMALL1);
    LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
    LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L;
}}

int main(void) {
    prvSetupHardware();

    uart = new LpcUart{ { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, { 0, 18 }, { 0, 13 } } };

#if EX1

    DigitalIOPin* ioButton1 = new DigitalIOPin{ { 0, 17 }, true, true, true, PIN_INT0_IRQn };
    DigitalIOPin* ioButton2 = new DigitalIOPin{ { 1, 11 }, true, true, true, PIN_INT1_IRQn };
    DigitalIOPin* ioButton3 = new DigitalIOPin{ { 1,  9 }, true, true, true, PIN_INT2_IRQn };

    auto const taskButtonHandler = [](void* pvParameters) {
        static size_t xTasksCreated{ 1 };
        size_t xTaskID = xTasksCreated++;

        auto& ioButton = *reinterpret_cast<DigitalIOPin*>(pvParameters);

        while (true) {
            ioButton.WFI(true, portMAX_DELAY);
            uart->print("SW%d pressed\r\n", xTaskID);
        }
    };

    xTaskCreate(taskButtonHandler, "Task 1", configMINIMAL_STACK_SIZE + 128, ioButton1, tskIDLE_PRIORITY + 1UL, nullptr);
    xTaskCreate(taskButtonHandler, "Task 2", configMINIMAL_STACK_SIZE + 128, ioButton2, tskIDLE_PRIORITY + 1UL, nullptr);
    xTaskCreate(taskButtonHandler, "Task 3", configMINIMAL_STACK_SIZE + 128, ioButton3, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX2 == 1

    static SemaphoreHandle_t xSemaphore = xSemaphoreCreateBinary();

    xTaskCreate([](void* pvParameters) {
        char c;

        while (true) {
            uart->read(&c, 1, portMAX_DELAY);
            uart->write(c);
            xSemaphoreGive(xSemaphore);
        }
    }, "Task 1", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](void* pvParameters) {
        while (true) {
            xSemaphoreTake(xSemaphore, portMAX_DELAY);
            Board_LED_Toggle(1);
            vTaskDelay(100);
            Board_LED_Toggle(1);
            vTaskDelay(100);
        }
    }, "Task 2", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX3 == 1

    static SemaphoreHandle_t xSemaphore = xSemaphoreCreateCounting(5, 0);

    xTaskCreate([](auto) {
        constexpr auto kBufferSize{ 61 };
        char buffer[kBufferSize];
        size_t count{ 0 };
        bool isQuestion = false;
        auto speakToOracle = [](auto begin, bool isQuestion) {
            uart->print("\r[You] %s\r\n", begin);
            if (isQuestion)
                xSemaphoreGive(xSemaphore);
        };

        while (true) {
            uart->read(&buffer[count], 1, portMAX_DELAY);
            switch (buffer[count]) {
            case '\n':
            case '\r':
                buffer[count] = '\0';
                speakToOracle(buffer, isQuestion);
                isQuestion = false;
                count = 0;
                break;

            case '?':
                isQuestion = true;
            default:
                uart->write(buffer[count]);
                if (++count == kBufferSize - 1) {
                    buffer[count] = '\0';
                    speakToOracle(buffer, isQuestion);
                    isQuestion = false;
                    count = 0;
                }
                break;
            }

        }
    }, "Task 1", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

    xTaskCreate([](auto) {
        constexpr std::array replies{ "Interesting...", "Whatever.", "Ask me again later.", "I think you're right about that.", "Oh, were you talking to me?" }; // Won't compile without C++17
        while (true) {
            xSemaphoreTake(xSemaphore, portMAX_DELAY);
            uart->write("\r[Oracle] Hmmm...\r\n");
            vTaskDelay(3000);
            uart->print("\r[Oracle] %s\r\n", replies[std::rand() % replies.size()]);
            vTaskDelay(2000);
        }
    }, "Task 2", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#endif

    vTaskStartScheduler();

    return 1;
}

