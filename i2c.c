/*
 * i2c.c
 *
 *  Created on: 2012. 8. 7.
 *      Author: East
 */
#include "i2c.h"

#include <stdlib.h>

#include "CMSIS/stm32f10x_gpio.h"
#include "CMSIS/stm32f10x_rcc.h"
#include "CMSIS/stm32f10x_i2c.h"
#include "CMSIS/misc.h"

#include "system_timer.h"

#define I2C_TIMEOUT_US 100

typedef enum I2cState
{
	I2C_ERROR_STATE, I2C_COMPLETE_STATE, I2C_ON_GOING_STATE
} I2cState;

typedef struct I2cDataContainer
{
	uint16_t address;
	uint8_t* data;
	uint32_t size;

	uint8_t direction;
	volatile uint32_t index;
	volatile I2cState state;
} I2cDataContainer;

static I2cDataContainer i2c2DataContainer =
{ 0, NULL, 0, I2C_Direction_Transmitter, 0, I2C_COMPLETE_STATE };

void I2C2_EV_IRQHandler()
{
	switch (I2C_GetLastEvent(I2C2))
	{
	case I2C_EVENT_MASTER_MODE_SELECT:
		I2C_Send7bitAddress(I2C2, (uint8_t) (i2c2DataContainer.address << 1),
				i2c2DataContainer.direction);
		break;

	case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
		i2c2DataContainer.index = 0;
		I2C_SendData(I2C2, i2c2DataContainer.data[i2c2DataContainer.index]);
		break;

	case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
		break;

	case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
		i2c2DataContainer.index++;
		if (i2c2DataContainer.index < i2c2DataContainer.size)
		{
			I2C_SendData(I2C2, i2c2DataContainer.data[i2c2DataContainer.index]);
		}
		else if (i2c2DataContainer.index == i2c2DataContainer.size)
		{
			I2C_GenerateSTOP(I2C2, ENABLE);
			i2c2DataContainer.state = I2C_COMPLETE_STATE;
		}
		break;

		///////////////////////////////////////////////////////////

	case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
		i2c2DataContainer.data[i2c2DataContainer.index++] = I2C_ReceiveData(
				I2C2);
		break;

	case I2C_EVENT_MASTER_BYTE_RECEIVED:
		if (i2c2DataContainer.index < i2c2DataContainer.size)
		{
			if (i2c2DataContainer.index + 1 == i2c2DataContainer.size)
			{
				I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Current);
			}
			i2c2DataContainer.data[i2c2DataContainer.index++] = I2C_ReceiveData(
					I2C2);
		}
		else
		{
			I2C_GenerateSTOP(I2C2, ENABLE);
			i2c2DataContainer.state = I2C_COMPLETE_STATE;
		}
		break;
	}
}

void I2C2_ER_IRQHandler()
{
	if (I2C_GetITStatus(I2C2, I2C_IT_BERR))
	{
		i2c2DataContainer.state = I2C_ERROR_STATE;
		I2C_ClearITPendingBit(I2C2, I2C_IT_BERR);
	}

	if (I2C_GetITStatus(I2C2, I2C_IT_AF))
	{
		i2c2DataContainer.state = I2C_ERROR_STATE;
		I2C_GenerateSTOP(I2C2, ENABLE);
		I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
	}

	if (I2C_GetITStatus(I2C2, I2C_IT_ARLO))
	{
		i2c2DataContainer.state = I2C_ERROR_STATE;
		I2C_ClearITPendingBit(I2C2, I2C_IT_ARLO);
	}

	if (I2C_GetITStatus(I2C2, I2C_IT_OVR))
	{
		i2c2DataContainer.state = I2C_ERROR_STATE;
		I2C_ClearITPendingBit(I2C2, I2C_IT_OVR);
	}
}

BOOL I2c2WriteRequest(uint16_t address, uint8_t* data, uint32_t size)
{
	uint32_t systemTickCount = GetSystemTickCount();

	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)
			|| i2c2DataContainer.state == I2C_ON_GOING_STATE)
	{
		if (TimeInterval(systemTickCount) > I2C_TIMEOUT_US)
		{
			i2c2DataContainer.state = I2C_ERROR_STATE;
			return FALSE;
		}
	}

	i2c2DataContainer.state = I2C_ON_GOING_STATE;
	i2c2DataContainer.address = address;
	i2c2DataContainer.data = data;
	i2c2DataContainer.size = size;
	i2c2DataContainer.index = 0;
	i2c2DataContainer.direction = I2C_Direction_Transmitter;

	I2C_GenerateSTART(I2C2, ENABLE);
	return TRUE;
}

BOOL I2c2ReadRequest(uint16_t address, uint8_t* data, uint32_t availableSize)
{
	uint32_t systemTickCount = GetSystemTickCount();
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)
			|| i2c2DataContainer.state == I2C_ON_GOING_STATE)
	{
		if (TimeInterval(systemTickCount) > I2C_TIMEOUT_US)
		{
			i2c2DataContainer.state = I2C_ERROR_STATE;
			return FALSE;
		}
	}

	i2c2DataContainer.state = I2C_ON_GOING_STATE;
	i2c2DataContainer.address = address;
	i2c2DataContainer.data = data;
	i2c2DataContainer.size = availableSize;
	i2c2DataContainer.index = 0;
	i2c2DataContainer.direction = I2C_Direction_Receiver;

	I2C_GenerateSTART(I2C2, ENABLE);
	return TRUE;
}

BOOL IsI2c2OnGoing()
{
	return i2c2DataContainer.state == I2C_ON_GOING_STATE;
}

BOOL IsI2c2Completed()
{
	return i2c2DataContainer.state == I2C_COMPLETE_STATE;
}

BOOL IsI2c2Error()
{
	return i2c2DataContainer.state == I2C_ERROR_STATE;
}

void I2cRccConfig()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void I2cNvicConfig()
{
	NVIC_InitTypeDef nvicI2cInit;

	nvicI2cInit.NVIC_IRQChannel = I2C2_EV_IRQn;
	nvicI2cInit.NVIC_IRQChannelPreemptionPriority = 2;
	nvicI2cInit.NVIC_IRQChannelSubPriority = 0;
	nvicI2cInit.NVIC_IRQChannelCmd = ENABLE;

	I2C_ITConfig(I2C2, I2C_IT_EVT, DISABLE);
	NVIC_Init(&nvicI2cInit);

	nvicI2cInit.NVIC_IRQChannel = I2C2_ER_IRQn;
	nvicI2cInit.NVIC_IRQChannelPreemptionPriority = 2;
	nvicI2cInit.NVIC_IRQChannelSubPriority = 0;
	nvicI2cInit.NVIC_IRQChannelCmd = ENABLE;

	I2C_ITConfig(I2C2, I2C_IT_ERR, DISABLE);
	NVIC_Init(&nvicI2cInit);
}

void I2cInit(uint32_t clockSpeed)
{
	GPIO_InitTypeDef gpioI2cInit;

	/* Configure I2C2 pins: SCL and SDA ----------------------------------------*/
	gpioI2cInit.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpioI2cInit.GPIO_Speed = GPIO_Speed_50MHz;
	gpioI2cInit.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &gpioI2cInit);

	I2C_InitTypeDef i2cInit;
	/* I2C2 configuration: SMBus Host ------------------------------------------*/
	i2cInit.I2C_Mode = I2C_Mode_I2C;
	i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
	i2cInit.I2C_OwnAddress1 = 0;
	i2cInit.I2C_Ack = I2C_Ack_Enable;
	i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2cInit.I2C_ClockSpeed = clockSpeed;
	I2C_Init(I2C2, &i2cInit);

	I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

	I2C_Cmd(I2C2, ENABLE);
}

