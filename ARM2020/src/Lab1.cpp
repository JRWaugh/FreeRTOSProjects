#include "board.h"
#include "FreeRTOS.h"
#include "heap_lock_monitor.h"
#include "DigitalIOPin.h"
#include "semphr.h"
#include <array>
#include <algorithm>

#define EX1 0
#define EX2 1
#define EX3 1

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

#if EX1

	xTaskCreate([](auto) {
		size_t ticks = 0;

		while (true) {
			DEBUGOUT("Time: %02d:%02d\r\n", ticks / 60, ticks % 60);
			++ticks;
			vTaskDelay(configTICK_RATE_HZ);
		}
	}, "vTaskEX1Uart1", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX2

	static SemaphoreHandle_t xSemaphore = xSemaphoreCreateBinary();

	xTaskCreate([](auto) {
		using MorseChar_t = std::array<TickType_t, 5>; // Morse character can have up to 5 dots and dashes in its sequence
		constexpr auto DOT = configTICK_RATE_HZ / 5;
		constexpr auto DASH = DOT * 3 - DOT;
		constexpr auto SPACE = DOT * 7 - DASH - DOT;
		constexpr MorseChar_t S = { DOT, DOT, DOT };
		constexpr MorseChar_t O = { DASH, DASH, DASH };
		constexpr std::array morseSOS = { S, O, S }; // Won't compile without C++17.
		constexpr auto blinkMorseChar = [](MorseChar_t const& morse_char) {
			std::find_if(morse_char.begin(), morse_char.end(), [](auto blip) {
				if (blip != 0) {
					Board_LED_Set(0, true);
					vTaskDelay(blip);
					Board_LED_Set(0, false);
					vTaskDelay(DOT);
					return false;
				} else {
					vTaskDelay(DASH);
					return true;
				}
			});
		};

		while (true) {
			for (auto const& morse_char : morseSOS)
				blinkMorseChar(morse_char);
			vTaskDelay(SPACE);
			xSemaphoreGive(xSemaphore);
		}
	}, "vTaskEX2Led1", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	/* LED2 toggle thread */
	xTaskCreate([](auto) {
		while (true) {
			xSemaphoreTake(xSemaphore, portMAX_DELAY);
			Board_LED_Toggle(1);
		}
	}, "vTaskEX2Led2", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#elif EX3

	xTaskCreate([](auto) {
	    static volatile size_t kFrequency{ 1 };
		size_t ticks{ 0 };
		DigitalIOPin SW1{ { 0, 17 }, true, true, true, PIN_INT0_IRQn, [](bool pressed) {
		    if (pressed)
		        kFrequency = 10;
		    else
		        kFrequency = 1;
		}};

		while (true) {
			DEBUGOUT("Tick: %d\r\n", ticks++);
			vTaskDelay(configTICK_RATE_HZ / kFrequency);
		}
	}, "vTaskEX3Uart1", configMINIMAL_STACK_SIZE + 128, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

#endif
	vTaskStartScheduler();
}
