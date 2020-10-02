#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ITM_write.h"
#include "user_vcom.h"
#include "heap_lock_monitor.h"
#include <cstring>
#include <array>
#include "circularbuffer.h"
#include "UART.h"
#include <type_traits>

#define EX1 0
#define EX2 0
#define EX3 1

extern "C" {
void vConfigureTimerForRunTimeStats() {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}
}

/* Sets up system hardware */
static void prvSetupHardware() {
	SystemCoreClockUpdate();
	Board_Init();
	heap_monitor_setup();
	ITM_init();
}

#if EX1 == 1 || EX2 == 1

static void send_task(void* pvParameters) {
#if EX1 == 1
	uint32_t count = 0;
	char str[32];
#elif EX2 == 1
	char str[] = "Hello World!\r\n";
#endif

	vTaskDelay(100); // wait until semaphores are created
	while (true) {
#if EX1 == 1
		int len = sprintf(str, "Counter: %lu runs really fast\r\n", count++);
		USB_send((uint8_t *) str, len);
#elif EX2 == 1
		USB_send((uint8_t *) str, sizeof(str));
#endif
		Board_LED_Toggle(0);

		vTaskDelay(configTICK_RATE_HZ / 50);
	}
}

static void receive_task(void* pvParameters) {
	char str[80];

	vTaskDelay(100); // wait until semaphores are created
	while (true) {
		uint32_t len = USB_receive((uint8_t *) str, 79);
		str[len] = 0;
		ITM_write(str);

		Board_LED_Toggle(1);
	}
}

#elif EX3 == 1

/* Bind a parameter to a FreeRTOS task with compile-time type checking */
template <typename F, typename T>
void xTaskCreateWithBind(
		F&& f, T parameter, char const * const name,
		configSTACK_DEPTH_TYPE stack_size = configMINIMAL_STACK_SIZE,
		UBaseType_t priority = tskIDLE_PRIORITY + 1UL,
		TaskHandle_t * const created_task = nullptr) {
	static_assert(std::is_invocable<F, T>::value);

	xTaskCreate((TaskFunction_t) +f, name, stack_size, parameter, priority, created_task);
}

static void USBSend(char const * data) {
	USB_send((uint8_t *) data, strlen(data));
}

static void usb_sender(SemaphoreHandle_t semaphore) {
	char rcv_buffer[RCV_BUFSIZE + 1];
	char snd_buffer[61];
	size_t count{ 0 };
	bool isQuestion{ false };

	vTaskDelay(100); // wait until semaphores are created
	while (true) {
		auto len = USB_receive((uint8_t *) rcv_buffer, RCV_BUFSIZE);

		for (size_t i = 0; i < len; ++i) { // Copy received bytes into buffer while checking for special characters and length
			switch (rcv_buffer[i]) {
			case '\n':
			case '\r':
				snd_buffer[count] = '\0';
				USBSend("\r[You] ");
				USBSend(snd_buffer);
				USBSend("\r\n");
				count = 0;
				if (isQuestion) {
					xSemaphoreGive(semaphore);
					isQuestion = false;
				}
				break;

			case '?':
				isQuestion = true;
			default:
				USB_send((uint8_t *) &rcv_buffer[i], 1);
				snd_buffer[count++] = rcv_buffer[i];

				if (count == 60) { // Buffer is full
					snd_buffer[count] = '\0';
					USBSend("\r[You] ");
					USBSend(snd_buffer);
					USBSend("\r\n");
					count = 0;
					if (isQuestion) {
						xSemaphoreGive(semaphore);
						isQuestion = false;
					}
				}
				break;
			}
		}
	}
}

static void usb_replyer(SemaphoreHandle_t semaphore) {
	constexpr std::array replies{ "Interesting...", "Whatever.", "Ask me again later.", "I think you're right about that.", "Oh, were you talking to me?" }; // Won't compile without C++17

	vTaskDelay(100); // wait until semaphores are created
	while (true) {
		xSemaphoreTake(semaphore, portMAX_DELAY);
		USBSend("\r[Oracle] I find your lack of faith disturbing\r\n");
		vTaskDelay(3000);
		USBSend("\r[Oracle] ");
		USBSend(replies[std::rand() % replies.size()]);
		USBSend("\r\n");
		vTaskDelay(2000);
	}

}
static void command_reader(FreeRTOS::UART* uart) {
	circularbuffer<char, 4> command;
	char buffer[200] { 0 };

	vTaskDelay(100); // wait until semaphores are created
	while (true) {
		char c = uart->read();
		if (c == '\r' || c == '\n') {
			uart->write("\r\n");

			for (size_t i = 0; i < command.size(); ++i)
				buffer[i] = command[i];

			if (strstr(buffer, "list") != nullptr) {
				vTaskList(buffer);
				uart->write(buffer);
			} else if (strstr(buffer, "stat") != nullptr) {
				vTaskGetRunTimeStats(buffer);
				uart->write(buffer);
			}
		} else {
			uart->write(c);
			command.push_back(c);
		}
	}
}
#endif

int main(void) {
	prvSetupHardware();

#if EX1 == 1 || EX2 == 1
#if EX1 == 1
	/* LED1 toggle thread */
	xTaskCreate(send_task, "Tx", 165, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);
#elif EX2 == 1
	/* LED1 toggle thread */
	xTaskCreate(send_task, "Tx", 50, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);
#endif
	/* LED1 toggle thread */
	xTaskCreate(receive_task, "Rx", 65, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);
#endif

#if EX3 == 1

	SemaphoreHandle_t semaphore = xSemaphoreCreateCounting(5, 0);
	FreeRTOS::UART* uart = new FreeRTOS::UART({ LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false,
		{ 0, 18 }, { 0, 13 } });

	xTaskCreateWithBind(usb_sender, semaphore, "USB Sender", 90);
	xTaskCreateWithBind(usb_replyer, semaphore, "USB Replyer", 60);
	xTaskCreateWithBind(command_reader, uart, "Command Reader", 225);

#endif

	/* LED2 toggle thread */
	xTaskCreate(cdc_task, "CDC", 93, nullptr, tskIDLE_PRIORITY + 1UL, nullptr);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
