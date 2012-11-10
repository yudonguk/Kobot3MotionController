/*
 * system_timer.c
 *
 *  Created on: 2012. 8. 7.
 *      Author: East
 */

#include "CMSIS/stm32f10x.h"

static volatile uint32_t sysTickCount = 0;

void SysTick_Handler(void)
{
	sysTickCount++;
}

void SystemTimerInit()
{
	SysTick_Config(SystemCoreClock / 1000000); // 1us
	NVIC_SetPriority(SysTick_IRQn, 0);
}

void Delay_us(uint32_t us)
{
	uint32_t startTick = sysTickCount;

	while ((uint32_t) (sysTickCount - startTick) < us)
		;
}

void Delay_ms(uint32_t ms)
{
	uint32_t startTick = sysTickCount;
	uint32_t us = ms * 1000;

	while ((uint32_t) (sysTickCount - startTick) < us)
		;
}

uint32_t TimeInterval(uint32_t startSysTickCount)
{
	return (uint32_t) (sysTickCount - startSysTickCount);
}

uint32_t GetSystemTickCount()
{
	return sysTickCount;
}
