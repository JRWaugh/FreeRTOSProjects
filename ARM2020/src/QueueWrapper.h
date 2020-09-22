/*
 * Queue.h
 *
 *  Created on: 7 Sep 2020
 *      Author: Joshua
 */

#ifndef FREERTOS_QUEUEWRAPPER_H_
#define FREERTOS_QUEUEWRAPPER_H_

#include "FreeRTOS.h"
#include "queue.h"

template <typename T, size_t S>
class QueueWrapper {
public:
	using value_type = T;

	QueueWrapper() : queue{ xQueueCreate(S, sizeof(T)) } {}

	~QueueWrapper() {
		vQueueDelete(queue);
	}

	QueueWrapper(QueueWrapper const & queue) = delete;

	BaseType_t push_front(T const & t, TickType_t ticksToWait = 0) {
		if (!is_interrupt())
			return xQueueSendToFront(queue, &t, ticksToWait);
		else {
			portBASE_TYPE xHigherPriorityWoken = pdFALSE;
			auto res = xQueueSendToFrontFromISR(queue, &t, &xHigherPriorityWoken);
			portEND_SWITCHING_ISR(xHigherPriorityWoken);
			return res;
		}
	}

	BaseType_t push_back(T const & t, TickType_t ticksToWait = 0) {
		if (!is_interrupt())
			return xQueueSendToBack(queue, &t, ticksToWait);
		else {
			portBASE_TYPE xHigherPriorityWoken = pdFALSE;
			auto res = xQueueSendToBackFromISR(queue, &t, &xHigherPriorityWoken);
			portEND_SWITCHING_ISR(xHigherPriorityWoken);
			return res;
		}
	}

	// Dangerous function
	[[nodiscard]] T pop_back(TickType_t ticksToWait = portMAX_DELAY) {
		T t;
		if (!is_interrupt())
			xQueueReceive(queue, &t, ticksToWait);
		else {
			portBASE_TYPE xHigherPriorityWoken = pdFALSE;
			xQueueSendToBackFromISR(queue, &t, &xHigherPriorityWoken);
			portEND_SWITCHING_ISR(xHigherPriorityWoken);
		}
		return t;
	}

	[[nodiscard]] T peek(TickType_t ticksToWait = portMAX_DELAY) {
		T t;
		if (!is_interrupt())
			xQueuePeek(queue, &t, ticksToWait);
		else
			xQueuePeekFromISR(queue, &t);
		return t;
	}

	[[nodiscard]] bool empty() {
		if (!is_interrupt())
			return uxQueueMessagesWaiting(queue);
		else
			return xQueueIsQueueEmptyFromISR(queue);
	}

	[[nodiscard]] size_t size() {
		if (!is_interrupt())
			return uxQueueMessagesWaiting(queue);
		else
			return uxQueueMessagesWaitingFromISR(queue);
	}

private:
	QueueHandle_t queue;

	[[nodiscard]] static bool is_interrupt() {
		return SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
	}
};
#endif /* FREERTOS_QUEUEWRAPPER_H_ */
