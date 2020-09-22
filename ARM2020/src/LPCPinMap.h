/*
 * LPCPinMap.h
 *
 *  Created on: 10 Sep 2020
 *      Author: Joshua
 */

#ifndef LPCPINMAP_H_
#define LPCPINMAP_H_

#include <cstdint>

struct LPCPinMap {
	int8_t port; /* set to -1 to indicate unused pin */
	int8_t pin;  /* set to -1 to indicate unused pin */
};

#endif /* LPCPINMAP_H_ */
