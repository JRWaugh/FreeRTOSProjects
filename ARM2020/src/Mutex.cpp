/*
 * Fmutex.cpp
 *
 *  Created on: 25 Aug 2020
 *      Author: Joshua
 */

#include "Mutex.h"

namespace FreeRTOS {
Mutex::Mutex() {
	mutex = xSemaphoreCreateMutex();
}

Mutex::~Mutex() {
	vSemaphoreDelete(mutex);
}

void Mutex::lock() {
	xSemaphoreTake(mutex, portMAX_DELAY);
}

void Mutex::unlock() {
	xSemaphoreGive(mutex);
}
}
