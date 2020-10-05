/*
 * Lab10.cpp
 *
 *  Created on: 5 Oct 2020
 *      Author: Joshua
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "heap_lock_monitor.h"
#include "DigitalIOPin.h"
#include "QueueWrapper.h"
#include "semphr.h"
#include "LpcUart.h"
#include "timers.h"
#include <cstring>

#define EX1 1
#define EX2 0
#define EX3 0

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

#if EX1

	static QueueWrapper<char const *, 6>* xQueue = new QueueWrapper<char const *, 6>();

	xTimerStart(xTimerCreate("Auto Reload", configTICK_RATE_HZ * 5, pdTRUE, nullptr, [](TimerHandle_t xTimer) { xQueue->push_back("Hello!\r\n"); }), 0);

	xTaskCreate([](void* pvParameters) {
		LpcUart uart( { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, { 0, 18 }, { 0, 13 } } );

		while (true)
			uart.write(xQueue->pop_front());
	}, "Task 1", configMINIMAL_STACK_SIZE + 200, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	xTaskCreate([](void* pvParameters) {
		static SemaphoreHandle_t xSemaphore = xSemaphoreCreateBinary();
		xTimerStart(xTimerCreate("One Shot",  configTICK_RATE_HZ * 20, pdFALSE, nullptr, [](TimerHandle_t xTimer) { xSemaphoreGive(xSemaphore); }), 0);

		while (true) {
			xSemaphoreTake(xSemaphore, portMAX_DELAY);
			xQueue->push_back("Aargh!\r\n");
		}
	}, "Task 2", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX2

	static TimerHandle_t xTimer = xTimerCreate("Reset LED Timer", configTICK_RATE_HZ * 5, pdFALSE, nullptr, [](TimerHandle_t xTimer) {
		Board_LED_Set(1, false);
	});

	auto const callback = [](bool pressed) {
		if (pressed) {
			Board_LED_Set(1, true);

			portBASE_TYPE xHigherPriorityWoken = pdFALSE;
			xTimerResetFromISR(xTimer, &xHigherPriorityWoken);
			portEND_SWITCHING_ISR(xHigherPriorityWoken);
		}
	};

	DigitalIOPin* ioButton1 = new DigitalIOPin{{ 0, 17 }, true, true, true, PIN_INT0_IRQn, callback };
	DigitalIOPin* ioButton2 = new DigitalIOPin{{ 1, 11 }, true, true, true, PIN_INT1_IRQn, callback };
	DigitalIOPin* ioButton3 = new DigitalIOPin{{ 1,  9 }, true, true, true, PIN_INT2_IRQn, callback };

#elif EX3

	xTaskCreate([](void* pvParameters) {
		static size_t count{ 0 };
		static LpcUart* uart = new LpcUart{ { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, { 0, 18 }, { 0, 13 } } };
		char buffer[60];
		size_t interval;

		TimerHandle_t xTimer = xTimerCreate("Green LED Toggle", configTICK_RATE_HZ * 5, pdTRUE, nullptr, [](TimerHandle_t xTimer) {
			Board_LED_Toggle(1);
		});
		xTimerStart(xTimer, 0);

		TimerHandle_t xInactivityTimer = xTimerCreate("Inactivity Monitor", configTICK_RATE_HZ * 30, pdFALSE, nullptr, [](TimerHandle_t xTimer) {
			uart->write("\r\n[Inactive]\r\n");
			count = 0;
		});
		xTimerStart(xInactivityTimer, 0);

		while (true) {
			uart->read(&buffer[count], 1, portMAX_DELAY);
			xTimerReset(xInactivityTimer, 0);

			if (buffer[count] == '\r' || buffer[count] == '\n') {
				uart->write("\r\n");
				buffer[count] = '\0';
				if (char* res = strstr(buffer, "help"); res != nullptr) {
					uart->write(
							"[ INSTRUCTIONS ]\r\n"
							"This program controls the period between LED toggles.\r\n"
							"To change the period, use the command \"interval <number>\", \r\n"
							"where the number is milliseconds between toggles.\r\n"
							"To see when the last toggle occurred, using the command \"time\". \r\n"
							"The time will be given with a 0.1s accuracy.\r\n");
				} else if (char* res = strstr(buffer, "interval "); res != nullptr) {
					if (sscanf(res + 9, "%d", &interval) == 1)
						xTimerChangePeriod(xTimer, interval, 0);
				} else if (char* res = strstr(buffer, "time"); res != nullptr) {
					uart->print("Time since last toggle: %0.1fs\r\n", ( xTaskGetTickCount() - xTimerGetExpiryTime(xTimer) + xTimerGetPeriod(xTimer) ) / (float) configTICK_RATE_HZ);
				}
				count = 0;
			}
			else {
				uart->write(buffer[count]);
				if (buffer[count] == 127 && count != 0)  // "Backspace", encoded as DEL by Putty
					--count;
				else if (++count == 60) {
					count = 0;
					uart->write("\r\nBuffer full!\r\n");
				}
			}
		}
	}, "Task 1", configMINIMAL_STACK_SIZE + 256, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#endif

	vTaskStartScheduler();
}
