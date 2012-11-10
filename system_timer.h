/*
 * system_timer.h
 *
 *  Created on: 2012. 8. 7.
 *      Author: East
 */

#ifndef SYSTEM_TIMER_H_
#define SYSTEM_TIMER_H_

#include "inttypes.h"

void SystemTimerInit();

uint32_t TimeInterval(uint32_t startSysTickCount);
uint32_t GetSystemTickCount();

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);

#endif /* SYSTEM_TIMER_H_ */
