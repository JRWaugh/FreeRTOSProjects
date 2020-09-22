/*
 * DigitalIoPin.cpp
 *
 *  Created on: 10 Mar 2020
 *      Author: Joshua
 */
#include "DigitalIOPin.h"

static DigitalIOPin* io[8]{ nullptr };

extern "C" {
void PIN_INT0_IRQHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	if (io[0])
		io[0]->isr();
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void PIN_INT1_IRQHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	if (io[1])
		io[1]->isr();
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void PIN_INT2_IRQHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	if (io[2])
		io[2]->isr();
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void PIN_INT3_IRQHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(3));
	if (io[3])
		io[3]->isr();
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void PIN_INT4_IRQHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(4));
	if (io[4])
		io[4]->isr();
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void PIN_INT5_IRQHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(5));
	if (io[5])
		io[5]->isr();
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void PIN_INT6_IRQHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(6));
	if (io[6])
		io[6]->isr();
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

void PIN_INT7_IRQHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(7));
	if (io[7])
		io[7]->isr();
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}
}

DigitalIOPin::DigitalIOPin(LPCPinMap pin_map, bool input, bool pullup, bool invert, IRQn_Type IRQn, onIRQCallback callback)
: pin_map{ pin_map }, channel{ IRQn - kIRQnMin }, invert{ invert }, IRQn{ IRQn }, callback{ callback } {

	LPC_IOCON->PIO[pin_map.port][pin_map.pin] = (1U + pullup) << 3 | 1U << 7 | invert << 6;
	LPC_GPIO->DIR[pin_map.port] = input ? LPC_GPIO->DIR[pin_map.port] & ~(1UL << pin_map.pin) : LPC_GPIO->DIR[pin_map.port] | 1UL << pin_map.pin;

	if (IRQn != kNoIRQ) {
		if (!isInit) {
			Chip_PININT_Init(LPC_GPIO_PIN_INT);
			isInit = true;
		}
		io[channel] = this;
		Chip_INMUX_PinIntSel(channel, pin_map.port, pin_map.pin);
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(channel));

		Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(channel));
		Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(channel));
		Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(channel));

		NVIC_ClearPendingIRQ(IRQn);
		NVIC_EnableIRQ(IRQn);
	}
}

DigitalIOPin::~DigitalIOPin() {
	NVIC_DisableIRQ(IRQn);

	Chip_PININT_DisableIntHigh(LPC_GPIO_PIN_INT, PININTCH(channel));
	Chip_PININT_DisableIntLow(LPC_GPIO_PIN_INT, PININTCH(channel));

	io[channel] = nullptr;
}

bool DigitalIOPin::read() const {
	return static_cast<bool>(LPC_GPIO->B[pin_map.port][pin_map.pin]);
}

void DigitalIOPin::write(bool const value) {
	LPC_GPIO->B[pin_map.port][pin_map.pin] = invert ? !value : value;
}

bool DigitalIOPin::toggle() {
	bool state = !read();
	write(state);
	return state;
}

void DigitalIOPin::setOnIRQCallback(onIRQCallback callback) {
	this->callback = callback;
}

void DigitalIOPin::isr() {
	if (callback != nullptr)
		callback(read());
}

bool DigitalIOPin::isInit{ false };
