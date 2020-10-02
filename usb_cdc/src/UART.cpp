/*
 * LpcUart.cpp
 *
 *  Created on: 4.2.2019
 *      Author: keijo
 */

#include <cstring>

#include "UART.h"

static FreeRTOS::UART* u0;
static FreeRTOS::UART* u1;
static FreeRTOS::UART* u2;


extern "C" {
void UART0_IRQHandler(void) {
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
       code if you need more capability. */
	if (u0)
		u0->isr();
}

void UART1_IRQHandler(void) {
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
       code if you need more capability. */
	if (u1)
		u1->isr();
}

void UART2_IRQHandler(void) {
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
       code if you need more capability. */
	if (u2)
		u2->isr();
}
}

namespace FreeRTOS {
void UART::isr() {
	/* Handle transmit interrupt if enabled */
	if (uart->STAT & UART_STAT_TXRDY) {
		uint8_t ch;

		/* Fill FIFO until full or until TX ring buffer is empty */
		while (uart->STAT & UART_STAT_TXRDY && RingBuffer_Pop(&txring, &ch))
			Chip_UART_SendByte(uart, ch);

		/* Disable transmit interrupt if the ring buffer is empty */
		if (RingBuffer_IsEmpty(&txring))
			uart->INTENCLR = UART_INTEN_TXRDY;

		xSemaphoreGiveFromISR(write_complete, nullptr);

	}

	/* Handle receive interrupt */
	while (uart->STAT & UART_STAT_RXRDY) {
		uint8_t ch = Chip_UART_ReadByte(uart);
		RingBuffer_Insert(&rxring, &ch);
		xSemaphoreGiveFromISR(read_ready, nullptr);
	}
}

bool UART::isInit = false;

UART::UART(const LpcUartConfig &cfg) : uart{ cfg.pUART }, read_ready{ xSemaphoreCreateCounting(UART_RB_SIZE, 0) },
		write_complete{ xSemaphoreCreateBinary() } {
	if (!isInit) {
		/* Before setting up the UART, the global UART clock for USARTS 1-4
		 * must first be setup. This requires setting the UART divider and
		 * the UART base clock rate to 16x the maximum UART rate for all
		 * UARTs.
		 * */
		/* Use main clock rate as base for UART baud rate divider */
		Chip_Clock_SetUARTBaseClockRate(Chip_Clock_GetMainClockRate(), false);
		isInit = true;
	}

	if (cfg.tx.port >= 0) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, cfg.tx.port, cfg.tx.pin, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		if (cfg.pUART == LPC_USART0)
			Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, cfg.tx.port, cfg.tx.pin);
		else if (cfg.pUART == LPC_USART1)
			Chip_SWM_MovablePortPinAssign(SWM_UART1_TXD_O, cfg.tx.port, cfg.tx.pin);
		else if (cfg.pUART == LPC_USART1)
			Chip_SWM_MovablePortPinAssign(SWM_UART2_TXD_O, cfg.tx.port, cfg.tx.pin);
	}

	if (cfg.rx.port >= 0) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, cfg.rx.port, cfg.rx.pin, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		if (cfg.pUART == LPC_USART0)
			Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, cfg.rx.port, cfg.rx.pin);
		else if (cfg.pUART == LPC_USART1)
			Chip_SWM_MovablePortPinAssign(SWM_UART1_RXD_I, cfg.rx.port, cfg.rx.pin);
		else if (cfg.pUART == LPC_USART1)
			Chip_SWM_MovablePortPinAssign(SWM_UART2_RXD_I, cfg.rx.port, cfg.rx.pin);
	}

	if (cfg.cts.port >= 0) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, cfg.cts.port, cfg.cts.pin, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		if (cfg.pUART == LPC_USART0)
			Chip_SWM_MovablePortPinAssign(SWM_UART0_CTS_I, cfg.cts.port, cfg.cts.pin);
		else if (cfg.pUART == LPC_USART1)
			Chip_SWM_MovablePortPinAssign(SWM_UART1_CTS_I, cfg.cts.port, cfg.cts.pin);
	}

	if (cfg.rts.port >= 0) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, cfg.rts.port, cfg.rts.pin, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));

		if (cfg.rs485)
			uart->CFG |= (1 << 21) | (1 << 20);

		if (cfg.pUART == LPC_USART0)
			Chip_SWM_MovablePortPinAssign(SWM_UART0_RTS_O, cfg.rts.port, cfg.rts.pin);
		else if (cfg.pUART == LPC_USART1)
			Chip_SWM_MovablePortPinAssign(SWM_UART1_RTS_O, cfg.rts.port, cfg.rts.pin);
	}

	/* Setup UART */
	Chip_UART_Init(uart);
	Chip_UART_ConfigData(uart, cfg.data);
	Chip_UART_SetBaud(uart, cfg.speed);

	Chip_UART_Enable(uart);
	Chip_UART_TXEnable(uart);

	/* Before using the ring buffers, initialize them using the ring buffer init function */
	RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(uart, UART_INTEN_RXRDY);
	Chip_UART_IntDisable(uart, UART_INTEN_TXRDY);   /* May not be needed */

	/* Enable UART interrupt */
	if (uart == LPC_USART0) {
		u0 = this;
		irqn = UART0_IRQn;
	} else if (uart == LPC_USART1) {
		u1 = this;
		irqn = UART1_IRQn;
	} else if (uart == LPC_USART2) {
		u2 = this;
		irqn = UART2_IRQn;
	}
	NVIC_EnableIRQ(irqn);
}

UART::~UART() {
	if (uart != nullptr) {
		NVIC_DisableIRQ(irqn);
		Chip_UART_IntDisable(uart, UART_INTEN_RXRDY);
		Chip_UART_IntDisable(uart, UART_INTEN_TXRDY);

		if (uart == LPC_USART0)
			u0 = nullptr;
		else if (uart == LPC_USART1)
			u1 = nullptr;
		else if (uart == LPC_USART2)
			u2 = nullptr;
	}
}


int UART::free() noexcept {
	return RingBuffer_GetCount(&txring);;
}

int  UART::peek() noexcept {
	return RingBuffer_GetCount(&rxring);
}

char  UART::read() noexcept {
	xSemaphoreTake(read_ready, portMAX_DELAY); // Alert read() to wake up
	char c;
	Chip_UART_ReadRB(uart, &rxring, &c, 1);
	return c;
}

int UART::write(char c) noexcept {
	return write(&c, 1);
}

int UART::write(char const * buffer) noexcept {
	return write(buffer, strlen(buffer));
}

int UART::write(char const * buffer, int const len) noexcept {
	int res = Chip_UART_SendRB(uart, &txring, buffer, len);
	xSemaphoreTake(write_complete, portMAX_DELAY); // Block write() from returning until tx_ring has emptied
	return res;
}

void UART::speed(int bps) noexcept {
	Chip_UART_SetBaud(uart, bps);
}

bool UART::txempty() {
	return RingBuffer_GetCount(&txring) == 0;
}
}
