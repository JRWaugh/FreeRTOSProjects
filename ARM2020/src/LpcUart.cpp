/*
 * LpcUart.cpp
 *
 *  Created on: 4.2.2019
 *      Author: keijo
 */

#include <cstring>
#include <mutex>
#include "LpcUart.h"


static LpcUart* u0;
static LpcUart* u1;
static LpcUart* u2;

extern "C" {
void UART0_IRQHandler() {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (u0)
		u0->isr(&xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void UART1_IRQHandler() {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (u1)
		u1->isr(&xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void UART2_IRQHandler() {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (u2)
		u2->isr(&xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
}

void LpcUart::isr(portBASE_TYPE* hpw) {
	uint32_t istat = Chip_UART_GetIntStatus(uart); // get interrupt status for notifications

	Chip_UART_IRQRBHandler(uart, &rxring, &txring); // chip library is used to handle receive and transmit

	// notify of the events handled
	if (notify_rx && (istat & UART_STAT_RXRDY))
		vTaskNotifyGiveFromISR(notify_rx, hpw);

	if (notify_tx && (istat & UART_STAT_TXRDY))
		vTaskNotifyGiveFromISR(notify_tx, hpw);

	if (on_receive && (istat & UART_STAT_RXRDY))
		on_receive();
}

bool LpcUart::isInit{ false };

LpcUart::LpcUart(LpcUart::Config const & cfg) : uart{ cfg.pUART }, notify_rx{ nullptr }, notify_tx{ nullptr }, on_receive{ nullptr } {
	if (!isInit) {
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

	NVIC_SetPriority(irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	/* Enable UART interrupt */
	NVIC_EnableIRQ(irqn);
}

LpcUart::~LpcUart() {
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

void LpcUart::set_on_receive(void (*on_receive)()) {
	this->on_receive = on_receive;
}

int LpcUart::free() {
	std::lock_guard<FreeRTOS::Mutex> lock(write_mutex);

	return UART_RB_SIZE - RingBuffer_GetCount(&txring);
}

int LpcUart::peek() {
	std::lock_guard<FreeRTOS::Mutex> lock(read_mutex);

	return RingBuffer_GetCount(&rxring);
}

int LpcUart::read(char& c) {
	return read(&c, 1);
}

int LpcUart::read(char* buffer, int len) {
	std::lock_guard<FreeRTOS::Mutex> lock(read_mutex);

	if (RingBuffer_GetCount(&rxring) <= 0) {
		notify_rx = xTaskGetCurrentTaskHandle();
		while (RingBuffer_GetCount(&rxring) <= 0) {
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		}
		notify_rx = nullptr;
	}

	return Chip_UART_ReadRB(uart, &rxring, buffer, len);
}


int LpcUart::read(char* buffer, int len, TickType_t total_timeout, TickType_t ic_timeout) {
	std::lock_guard<FreeRTOS::Mutex> lock(read_mutex);

	// we can't read more than ring buffer size at a time
	if (len > UART_RB_SIZE)
		len = UART_RB_SIZE;

	TimeOut_t timeoutState;
	vTaskSetTimeOutState(&timeoutState);

	notify_rx = xTaskGetCurrentTaskHandle();
	while (RingBuffer_GetCount(&rxring) < len && xTaskCheckForTimeOut(&timeoutState, &total_timeout) == pdFALSE) {
		TickType_t timeout = total_timeout > ic_timeout ? ic_timeout : total_timeout;
		if (ulTaskNotifyTake( pdTRUE, timeout ) == 0)
			break;
	}
	notify_rx = nullptr;

	return Chip_UART_ReadRB(uart, &rxring, buffer, len);;
}

int LpcUart::write(char c) {
	return write(&c, 1);
}

int LpcUart::write(char const * s) {
	return write(s, strlen(s));
}

int LpcUart::write(char const * buffer, int len) {
	std::lock_guard<FreeRTOS::Mutex> lock(write_mutex);

	int pos = 0;
	notify_tx = xTaskGetCurrentTaskHandle();

	while (len > pos) {
		// restrict single write to ring buffer size
		int size = (len - pos) > UART_RB_SIZE ? UART_RB_SIZE : (len - pos);

		// wait until we have space in the ring buffer
		while (UART_RB_SIZE - RingBuffer_GetCount(&txring) < size) {
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		}
		pos += Chip_UART_SendRB(uart, &txring, buffer+pos, size);
	}

	notify_tx = nullptr;

	return pos;
}

void LpcUart::txbreak(bool brk) {
	// break handling not implemented yet
}

bool LpcUart::rxbreak() {
	// break handling not implemented yet
	return false;
}

void LpcUart::speed(int bps) {
	std::lock_guard<FreeRTOS::Mutex> lockw(write_mutex);
	std::lock_guard<FreeRTOS::Mutex> lockr(read_mutex);

	Chip_UART_SetBaud(uart, bps);
}

bool LpcUart::txempty() {
	std::lock_guard<FreeRTOS::Mutex> lock(write_mutex);

	return (RingBuffer_GetCount(&txring) == 0);
}
