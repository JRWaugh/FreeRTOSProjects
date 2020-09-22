/*
 * LpcUart.h
 *
 *  Created on: 4.2.2019
 *      Author: keijo
 */

#ifndef LPCUART_H_
#define LPCUART_H_

#include "chip.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Mutex.h"
#include "LPCPinMap.h"

class LpcUart {
public:
	struct Config {
		LPC_USART_T* pUART;
		uint32_t speed;
		uint32_t data;
		bool rs485;
		LPCPinMap tx{ -1, -1 };
		LPCPinMap rx{ -1, -1 };
		LPCPinMap rts{ -1, -1 }; // used as output enable if RS-485 mode is enabled
		LPCPinMap cts{ -1, -1 };
	};

	LpcUart(Config const & cfg);
	LpcUart(LpcUart const &) = delete;
	~LpcUart();
	int  free(); /* get amount of free space in transmit buffer */
	int  peek(); /* get number of received characters in receive buffer */
	int  write(char c);
	int  write(char const * s);
	int  write(char const * buffer, int len);
	int  read(char& c); /* get a single character. Returns number of characters read --> returns 0 if no character is available */
	int  read(char* buffer, int len);
	int  read(char* buffer, int len, TickType_t total_timeout, TickType_t ic_timeout = portMAX_DELAY);
	void txbreak(bool brk); /* set break signal on */
	bool rxbreak(); /* check if break is received */
	void speed(int bps); /* change transmission speed */
	bool txempty();
	void set_on_receive(void (*on_receive)());

	void isr(portBASE_TYPE* hpw); /* ISR handler. This will be called by the HW ISR handler. Do not call from application */

private:
	LPC_USART_T *uart;
	IRQn_Type irqn;
	static constexpr int UART_RB_SIZE = 128; // currently we support only fixed size ring buffers

	/* Transmit and receive ring buffers */
	RINGBUFF_T txring;
	RINGBUFF_T rxring;
	uint8_t rxbuff[UART_RB_SIZE];
	uint8_t txbuff[UART_RB_SIZE];

	TaskHandle_t notify_rx;
	TaskHandle_t notify_tx;
	void (*on_receive)(); // callback for received data notifications
	FreeRTOS::Mutex read_mutex;
	FreeRTOS::Mutex write_mutex;

	static bool isInit; /* set when first UART is initialized. We have a global clock setting for all UARTSs */
};

#endif /* LPCUART_H_ */
