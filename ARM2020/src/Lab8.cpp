/*
 * Lab8.cpp
 *
 *  Created on: 23 Sep 2020
 *      Author: Joshua
 */

#include "board.h"
#include "FreeRTOS.h"
#include "heap_lock_monitor.h"
#include "QueueWrapper.h"
#include "DigitalIOPin.h"
#include "LpcUart.h"
#include "semphr.h"
#include "timers.h"
#include <cstring>

#define EX1 0
#define EX2 0
#define EX3 1

struct IOTimestamp {
	BaseType_t which = 0;
	TickType_t when = 0;
};

LpcUart* uart;
QueueWrapper<IOTimestamp, 7>* queue;

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

	queue = new QueueWrapper<IOTimestamp, 7>;
	uart = new LpcUart{ { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, { 0, 18 }, { 0, 13 } } };

#if EX1

	xTaskCreate([](auto) {
		DigitalIOPin ioButton1{{ 0, 17 }, true, true, true, PIN_INT0_IRQn, [](bool pressed) {
			if (!pressed)
				queue->push_back({ 1, xTaskGetTickCountFromISR() });
		}};

		DigitalIOPin ioButton2{{ 1, 11 }, true, true, true, PIN_INT1_IRQn, [](bool pressed) {
			if (!pressed)
				queue->push_back({ 2, xTaskGetTickCountFromISR() });
		}};

		DigitalIOPin ioButton3{{ 1,  9 }, true, true, true, PIN_INT2_IRQn, [](bool pressed) {
			if (!pressed)
				queue->push_back({ 3, xTaskGetTickCountFromISR() });
		}};

		IOTimestamp previous = queue->pop_front(portMAX_DELAY);
		UBaseType_t count{ 1 };

		while (true) {
			IOTimestamp const current = queue->pop_front(portMAX_DELAY);
			if (current.which == previous.which)
				++count;
			else {
				uart->print("Button %ld pressed %ld times\r\n", previous.which, count);
				count = 1;
			}
			previous = current;
		}
	}, "Task 1",  configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX2

	static TickType_t filter{ 50 };

	xTaskCreate([](auto) {
		DigitalIOPin ioButton1{{ 0, 17 }, true, true, true, PIN_INT0_IRQn, [](bool pressed) {
			if (!pressed)
				queue->push_back({ 1, xTaskGetTickCountFromISR() });
		}};

		DigitalIOPin ioButton2{{ 1, 11 }, true, true, true, PIN_INT1_IRQn, [](bool pressed) {
			if (!pressed)
				queue->push_back({ 2, xTaskGetTickCountFromISR() });
		}};

		DigitalIOPin ioButton3{{ 1,  9 }, true, true, true, PIN_INT2_IRQn, [](bool pressed) {
			if (!pressed)
				queue->push_back({ 3, xTaskGetTickCountFromISR() });
		}};

		IOTimestamp previous = queue->pop_front();

		while (true) {
			IOTimestamp const current = queue->pop_front();
			TickType_t const delta = current.when - previous.when;

			if (delta > filter)
				uart->print("Button %ld was pressed %ld milliseconds after button %ld\r\n", current.which, delta, previous.which);

			previous = current;
		}
	}, "IO Monitor",  configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	xTaskCreate([](auto) {
		TickType_t value{ 0 };
		char buffer[60];
		auto iter = std::begin(buffer);

		while (true) {
			uart->read(iter, 1, portMAX_DELAY);
			if (*iter == '\r' || *iter == '\n') {
				uart->write("\r\n");
				*iter = '\0';
				if (char* res = strstr(buffer, "filter "); res != nullptr) {
					if (sscanf(res + 7, "%ld", &value) == 1) {
						filter = value;
						uart->print("New filter set: %ldms\r\n", filter);
					}
				}
				iter = std::begin(buffer);
			}
			else {
				uart->write(*iter);
				if (*iter == 127 && iter != std::begin(buffer))  // "Backspace", encoded as DEL by Putty
					--iter;
				else if (++iter == std::end(buffer)) {
					iter = std::begin(buffer);
					uart->write("\r\nBuffer full!\r\n");
				}
			}
		}
	}, "UART", configMINIMAL_STACK_SIZE + 200, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX3

	static TickType_t kDebounceTime{ 10 };

	static TimerHandle_t xBlueLEDHandler = xTimerCreate("Blue LED Handler", configTICK_RATE_HZ * 3, pdFALSE, nullptr, [](TimerHandle_t xTimer) {
		queue->push_back({ 2, xTaskGetTickCount() });
		Board_LED_Set(0, true);
		Board_LED_Set(1, false);
	});

	static TimerHandle_t xDoorLockTimer = xTimerCreate("Door Lock Timer", configTICK_RATE_HZ * 5, pdFALSE, nullptr, [](TimerHandle_t xTimer) {
		Board_LED_Set(0, true);
		Board_LED_Set(1, false);
	});

	xTaskCreate([](void* pvParameters) {
		DigitalIOPin ioButton1{{ 0, 17 }, true, true, true, PIN_INT0_IRQn, [](bool pressed) {
			static TickType_t previous{ 0 };
			TickType_t current = xTaskGetTickCountFromISR();

			if (pressed && current - previous > kDebounceTime)
				queue->push_back({ 0, current });

			previous = current;
		}};

		DigitalIOPin ioButton2{{ 1, 11 }, true, true, true, PIN_INT1_IRQn, [](bool pressed) {
			static TickType_t previous{ 0 };
			TickType_t current = xTaskGetTickCountFromISR();

			if (pressed && current - previous > kDebounceTime)
				queue->push_back({ 1, current });

			previous = current;
		}};

		DigitalIOPin io_button3{{ 1,  9 }, true, true, true, PIN_INT2_IRQn, [](bool pressed) {
			portBASE_TYPE xHigherPriorityWoken = pdFALSE;
			if (pressed)
				xTimerStartFromISR(xBlueLEDHandler, &xHigherPriorityWoken);
			else
				xTimerStopFromISR(xBlueLEDHandler, &xHigherPriorityWoken);
			portEND_SWITCHING_ISR(xHigherPriorityWoken);
		}};

		char input{ 0 };
		char password{ 0b11110110 };
		bool changing_password{ false };
		UBaseType_t count{ 0 };

		IOTimestamp previous;
		Board_LED_Set(0, true);

		while (true) {
			IOTimestamp const current = queue->pop_front();
			if (current.which == 2) {
				Board_LED_Set(0, false);
				Board_LED_Set(2, true);
				changing_password = true;
				count = 0;
			} else {
				if (!changing_password) {
					if (current.when - previous.when > 15000) // 15,000ms, or 15s
						input = 0;

					input <<= 1;
					input |= current.which;
					if (++count >= 8 && input == password) {
						Board_LED_Set(0, false);
						Board_LED_Set(1, true);
						xTimerStart(xDoorLockTimer, 0);
						count = 0;
					}
				} else {
					password <<= 1;
					password |= current.which;
					if (++count == 8) {
						Board_LED_Set(0, true);
						Board_LED_Set(2, false);
						changing_password = false;
						count = 0;
					}
				}
			}
			previous = current;
		}
	}, "IO Monitor", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#endif

	vTaskStartScheduler();

	return 1;
}
