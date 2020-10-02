/*
 * UART.h
 *
 *  Created on: 4.2.2019, modified on 9.9.2020
 *      Author: keijo, modified by Joshua
 */

#ifndef FREERTOS_UART_H_
#define FREERTOS_UART_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "chip.h"

namespace FreeRTOS {
class UART {
public:
	struct LpcPinMap {
		int8_t port; /* set to -1 to indicate unused pin */
		int8_t pin;  /* set to -1 to indicate unused pin */
	};

	struct LpcUartConfig {
		LPC_USART_T* pUART;
		uint32_t speed;
		uint32_t data;
		bool rs485;
		LpcPinMap tx{ -1, -1 };
		LpcPinMap rx{ -1, -1 };
		LpcPinMap rts{ -1, -1 }; // used as output enable if RS-485 mode is enabled
		LpcPinMap cts{ -1, -1 };
	};

	UART(const LpcUartConfig &cfg);
	UART(const UART &) = delete;
	~UART();
	int  free(); /* get amount of free space in transmit buffer */
	int  peek(); /* get number of received characters in receive buffer */
	int  write(char c);
	int  write(char const * buffer);
	int  write(char const * buffer, int len);
	char read(); /* get a single character. Returns number of characters read --> returns 0 if no character is available */
	void speed(int bps); /* change transmission speed */
	bool txempty();
	void isr(); /* ISR handler. This will be called by the HW ISR handler. Do not call from application */

private:
	LPC_USART_T* uart;
	IRQn_Type irqn;
	SemaphoreHandle_t read_ready, write_complete;
	static constexpr int UART_RB_SIZE = 128; // currently we support only fixed size ring buffers
	RINGBUFF_T txring;
	RINGBUFF_T rxring;
	uint8_t rxbuff[UART_RB_SIZE];
	uint8_t txbuff[UART_RB_SIZE];
	static bool isInit; /* set when first UART is initialized. We have a global clock setting for all UARTSs */
};
}

#endif /* FREERTOS_UART_H_ */
