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
#include "semphr.h"
#include "LPCPinMap.h"
#include <utility>

class DigitalIOPin {
public:
    using onIRQCallback = void (*)(bool pressed);

    DigitalIOPin(LPCPinMap pin_map, bool input, bool pullup, bool invert, IRQn_Type IRQn = kNoIRQ, onIRQCallback callback = nullptr);

    // Mustn't allow copies of a pin to exist
    DigitalIOPin(DigitalIOPin const &) = delete;
    DigitalIOPin& operator=(DigitalIOPin const &) = delete;
    DigitalIOPin(DigitalIOPin&& ioOld);
    ~DigitalIOPin();

    bool read() const;
    BaseType_t WFI(bool const value, TickType_t xBlockTime);
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
    SemaphoreHandle_t xSemaphore{ nullptr };

    static bool isInit;
    static constexpr size_t kDebounceTime{ 72 * 100000 };
    static constexpr IRQn_Type kNoIRQ	{ static_cast<IRQn_Type>(0) };
    static constexpr IRQn_Type kIRQnMin { PIN_INT0_IRQn };
    static constexpr IRQn_Type kIRQnMax { PIN_INT7_IRQn };
};
#endif /* DIGITALIOPIN_H_ */
