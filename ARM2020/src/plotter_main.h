/*
 * plotter_main.h
 *
 *  Created on: 17 Sep 2020
 *      Author: Joshua
 */

#ifndef PLOTTER_MAIN_H_
#define PLOTTER_MAIN_H_

namespace Plotter {
struct Message {
	enum Command { MoveLeft, MoveRight, SetPPS };
	Command command;
	BaseType_t value;
};

enum Direction {
	CounterClockwise = 0, Clockwise
};

enum Flag {
	Go = 1 << 0, CalibratingPPS = 1 << 1, PositionFound = 1 << 2, MaxWidthFound = 1 << 3, Initialised = 1 << 4
};

void vResume();
void vStop();
void vOnTick();
BaseType_t xGetPlotterWidth();
BaseType_t xEnqueueMessage(Message const & message);
BaseType_t xEnqueueMessage(Message::Command command, BaseType_t value);
void vInit(void (*startPulsing)(BaseType_t xPulsesPerSecond), void (*stopPulsing)());
}

#endif /* PLOTTER_MAIN_H_ */
