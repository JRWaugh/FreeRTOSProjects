/*
 * DigitalIoPin.h
 *
 *  Created on: 15 Jan 2020
 *      Author: Joshua
 */

#ifndef DIGITALIOPIN_H_
#define DIGITALIOPIN_H_

#include "board.h"
#include "FreeRTOS.h"
#include "LPCPinMap.h"

class DigitalIOPin {
public:
	using onIRQCallback = void (*)(bool pressed);

	DigitalIOPin(LPCPinMap pin_map, bool input, bool pullup, bool invert, IRQn_Type IRQn = kNoIRQ, onIRQCallback callback = nullptr);
	~DigitalIOPin();

	bool read() const;
	void write(bool const value);
	bool toggle();

	void setOnIRQCallback(onIRQCallback callback);
	void isr();

private:
	LPCPinMap const pin_map;
	int const channel;
	bool const invert;
	IRQn_Type IRQn;
	onIRQCallback callback;

	static bool isInit;
	static constexpr IRQn_Type kNoIRQ	{ static_cast<IRQn_Type>(0) };
	static constexpr IRQn_Type kIRQnMin { PIN_INT0_IRQn };
	static constexpr IRQn_Type kIRQnMax { PIN_INT7_IRQn };
};
#endif /* DIGITALIOPIN_H_ */
