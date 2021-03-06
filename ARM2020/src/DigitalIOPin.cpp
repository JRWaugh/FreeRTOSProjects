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
    if (io[0])
        io[0]->isr();
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
}

void PIN_INT1_IRQHandler() {
    if (io[1])
        io[1]->isr();
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
}

void PIN_INT2_IRQHandler() {
    if (io[2])
        io[2]->isr();
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
}

void PIN_INT3_IRQHandler() {
    if (io[3])
        io[3]->isr();
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(3));
}

void PIN_INT4_IRQHandler() {
    if (io[4])
        io[4]->isr();
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(4));
}

void PIN_INT5_IRQHandler() {
    if (io[5])
        io[5]->isr();
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(5));
}

void PIN_INT6_IRQHandler() {
    if (io[6])
        io[6]->isr();
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(6));
}

void PIN_INT7_IRQHandler() {
    if (io[7])
        io[7]->isr();
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(7));
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

        xSemaphore = xSemaphoreCreateBinary();

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

DigitalIOPin::DigitalIOPin(DigitalIOPin&& ioOld)
: pin_map{ ioOld.pin_map }, channel{ ioOld.channel }, invert{ ioOld.invert }, IRQn{ ioOld.IRQn }, callback{ ioOld.callback }, xSemaphore{ ioOld.xSemaphore } {
    io[channel] = this;
    ioOld.IRQn = kNoIRQ;
    ioOld.xSemaphore = nullptr;
}

DigitalIOPin::~DigitalIOPin() {
    if (IRQn != kNoIRQ) {
        NVIC_DisableIRQ(IRQn);

        Chip_PININT_DisableIntHigh(LPC_GPIO_PIN_INT, PININTCH(channel));
        Chip_PININT_DisableIntLow(LPC_GPIO_PIN_INT, PININTCH(channel));

        io[channel] = nullptr;
    }

    if (xSemaphore != nullptr)
        vSemaphoreDelete(xSemaphore);
}

bool DigitalIOPin::read() const {
    return static_cast<bool>(LPC_GPIO->B[pin_map.port][pin_map.pin]);
}


BaseType_t DigitalIOPin::WFI(bool const value, TickType_t xBlockTime) {
    TickType_t const xStartTicks = xTaskGetTickCount();
    if (xSemaphoreTake(xSemaphore, xBlockTime) == pdTRUE) {
        if (read() == value)
            return pdTRUE;
        else {
            xBlockTime = (xBlockTime == portMAX_DELAY) ? portMAX_DELAY : xBlockTime - (xTaskGetTickCount() - xStartTicks);
            return WFI(value, xBlockTime);
        }
    } else {
        return pdFALSE;
    }
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

    if (xSemaphore != nullptr) {
        portBASE_TYPE xHigherPriorityWoken = pdFALSE;
        xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityWoken);
        portEND_SWITCHING_ISR(xHigherPriorityWoken);
    }
}

bool DigitalIOPin::isInit{ false };
