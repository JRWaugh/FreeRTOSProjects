/*
 * Fmutex.h
 *
 *  Created on: 25 Aug 2020
 *      Author: Joshua
 */

#ifndef FREERTOS_MUTEX_H_
#define FREERTOS_MUTEX_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include <mutex>

namespace FreeRTOS {
class Mutex {
public:
	Mutex();
	~Mutex();
	void lock();
	void unlock();

private:
	SemaphoreHandle_t mutex;
};
}

#endif /* FREERTOS_MUTEX_H_ */
