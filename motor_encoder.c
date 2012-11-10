/*
 * motor_encoder.c
 *
 *  Created on: 2012. 8. 5.
 *      Author: East
 */
#include "motor_encoder.h"

#include "CMSIS/stm32f10x_rcc.h"
#include "CMSIS/stm32f10x_gpio.h"
#include "CMSIS/stm32f10x_exti.h"
#include "CMSIS/misc.h"

#define ENCODER_GPIO_PORT_INDEX			0
#define ENCODER_GPIO_PIN_INDEX			1
#define ENCODER_EXTI_PORT_SOURCE_INDEX	2
#define ENCODER_EXTI_PIN_SOURCE_INDEX	3
#define ENCODER_EXTI_LINE_INDEX			4
#define ENCODER_NVIC_IRQ_INDEX			5

#define ENCODER_REGISTER_ARRAY(channel, port, pin, irq) uint32_t channel[] = \
{\
	(uint32_t)GPIO ## port \
	, GPIO_Pin_ ## pin \
	, GPIO_PortSourceGPIO ## port \
	, GPIO_PinSource ## pin \
	, EXTI_Line ## pin \
	, irq \
}

ENCODER_REGISTER_ARRAY(ENCODER0_A_CHANNEL, A, 1, EXTI1_IRQn);
ENCODER_REGISTER_ARRAY(ENCODER0_B_CHANNEL, A, 0, EXTI0_IRQn);
ENCODER_REGISTER_ARRAY(ENCODER1_A_CHANNEL, C, 3, EXTI3_IRQn);
ENCODER_REGISTER_ARRAY(ENCODER1_B_CHANNEL, C, 2, EXTI2_IRQn);
ENCODER_REGISTER_ARRAY(ENCODER2_A_CHANNEL, A, 5, EXTI9_5_IRQn);
ENCODER_REGISTER_ARRAY(ENCODER2_B_CHANNEL, A, 4, EXTI4_IRQn);
ENCODER_REGISTER_ARRAY(ENCODER3_A_CHANNEL, A, 7, EXTI9_5_IRQn);
ENCODER_REGISTER_ARRAY(ENCODER3_B_CHANNEL, A, 6, EXTI9_5_IRQn);

#define ReadEncoderChannelLevel(channel)	GPIO_ReadInputDataBit((GPIO_TypeDef*)channel[ENCODER_GPIO_PORT_INDEX], channel[ENCODER_GPIO_PIN_INDEX])
#define GetEncoderGpioPort(channel)			((GPIO_TypeDef*)channel[ENCODER_GPIO_PORT_INDEX])
#define GetEncoderGpioPin(channel)			((uint16_t)channel[ENCODER_GPIO_PIN_INDEX])
#define GetEncoderExtiPortSource(channel)	((uint8_t)channel[ENCODER_EXTI_PORT_SOURCE_INDEX])
#define GetEncoderExtiPinSource(channel)	((uint8_t)channel[ENCODER_EXTI_PIN_SOURCE_INDEX])
#define GetEncoderExtiLine(channel)			(channel[ENCODER_EXTI_LINE_INDEX])
#define GetEncoderNvicIrqChannel(channel)	((uint8_t)channel[ENCODER_NVIC_IRQ_INDEX])

MotorEncoder motorEncoders[MOTOR_ENCODER_COUNT] =
{
{ 0, TRUE },
{ 0, TRUE },
{ 0, TRUE },
{ 0, TRUE } };

void EXTI0_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		uint32_t aChannel = ReadEncoderChannelLevel(ENCODER0_A_CHANNEL);
		uint32_t bChannel = Bit_SET;
		if (aChannel == bChannel)
		{
			//B상 인터럽트에서 aChannel == bChannel 일경우 정회전
			motorEncoders[MOTOR_ENCODER0_INDEX].encoderCount++;
			motorEncoders[MOTOR_ENCODER0_INDEX].isRotateFoward = TRUE;
		}
		else
		{
			//B상 인터럽트에서 aChannel != bChannel 일경우 역회전
			motorEncoders[MOTOR_ENCODER0_INDEX].encoderCount--;
			motorEncoders[MOTOR_ENCODER0_INDEX].isRotateFoward = FALSE;
		}

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI1_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		uint32_t aChannel = Bit_SET;
		uint32_t bChannel = ReadEncoderChannelLevel(ENCODER0_B_CHANNEL);
		if (aChannel == bChannel)
		{
			//A상 인터럽트에서 aChannel == bChannel 일경우 역회전
			motorEncoders[MOTOR_ENCODER0_INDEX].encoderCount--;
			motorEncoders[MOTOR_ENCODER0_INDEX].isRotateFoward = FALSE;
		}
		else
		{
			//A상 인터럽트에서 aChannel != bChannel 일경우 정회전
			motorEncoders[MOTOR_ENCODER0_INDEX].encoderCount++;
			motorEncoders[MOTOR_ENCODER0_INDEX].isRotateFoward = TRUE;
		}

		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI2_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		uint32_t aChannel = ReadEncoderChannelLevel(ENCODER1_A_CHANNEL);
		uint32_t bChannel = Bit_SET;
		if (aChannel == bChannel)
		{
			//B상 인터럽트에서 aChannel == bChannel 일경우 정회전
			motorEncoders[MOTOR_ENCODER1_INDEX].encoderCount++;
			motorEncoders[MOTOR_ENCODER1_INDEX].isRotateFoward = TRUE;
		}
		else
		{
			//B상 인터럽트에서 aChannel != bChannel 일경우 역회전
			motorEncoders[MOTOR_ENCODER1_INDEX].encoderCount--;
			motorEncoders[MOTOR_ENCODER1_INDEX].isRotateFoward = FALSE;
		}

		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void EXTI3_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		uint32_t aChannel = Bit_SET;
		uint32_t bChannel = ReadEncoderChannelLevel(ENCODER1_B_CHANNEL);
		if (aChannel == bChannel)
		{
			//A상 인터럽트에서 aChannel == bChannel 일경우 역회전
			motorEncoders[MOTOR_ENCODER1_INDEX].encoderCount--;
			motorEncoders[MOTOR_ENCODER1_INDEX].isRotateFoward = FALSE;
		}
		else
		{
			//A상 인터럽트에서 aChannel != bChannel 일경우 정회전
			motorEncoders[MOTOR_ENCODER1_INDEX].encoderCount++;
			motorEncoders[MOTOR_ENCODER1_INDEX].isRotateFoward = TRUE;
		}

		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

void EXTI4_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		uint32_t aChannel = ReadEncoderChannelLevel(ENCODER2_A_CHANNEL);
		uint32_t bChannel = Bit_SET;
		if (aChannel == bChannel)
		{
			//B상 인터럽트에서 aChannel == bChannel 일경우 정회전
			motorEncoders[MOTOR_ENCODER2_INDEX].encoderCount++;
			motorEncoders[MOTOR_ENCODER2_INDEX].isRotateFoward = TRUE;
		}
		else
		{
			//B상 인터럽트에서 aChannel != bChannel 일경우 역회전
			motorEncoders[MOTOR_ENCODER2_INDEX].encoderCount--;
			motorEncoders[MOTOR_ENCODER2_INDEX].isRotateFoward = FALSE;
		}

		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

void EXTI9_5_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		uint32_t aChannel = Bit_SET;
		uint32_t bChannel = ReadEncoderChannelLevel(ENCODER2_B_CHANNEL);
		if (aChannel == bChannel)
		{
			//A상 인터럽트에서 aChannel == bChannel 일경우 역회전
			motorEncoders[MOTOR_ENCODER2_INDEX].encoderCount--;
			motorEncoders[MOTOR_ENCODER2_INDEX].isRotateFoward = FALSE;
		}
		else
		{
			//A상 인터럽트에서 aChannel != bChannel 일경우 정회전
			motorEncoders[MOTOR_ENCODER2_INDEX].encoderCount++;
			motorEncoders[MOTOR_ENCODER2_INDEX].isRotateFoward = TRUE;
		}

		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		uint32_t aChannel = ReadEncoderChannelLevel(ENCODER3_A_CHANNEL);
		uint32_t bChannel = Bit_SET;
		if (aChannel == bChannel)
		{
			//B상 인터럽트에서 aChannel == bChannel 일경우 정회전
			motorEncoders[MOTOR_ENCODER3_INDEX].encoderCount++;
			motorEncoders[MOTOR_ENCODER3_INDEX].isRotateFoward = TRUE;
		}
		else
		{
			//B상 인터럽트에서 aChannel != bChannel 일경우 역회전
			motorEncoders[MOTOR_ENCODER3_INDEX].encoderCount--;
			motorEncoders[MOTOR_ENCODER3_INDEX].isRotateFoward = FALSE;
		}

		EXTI_ClearITPendingBit(EXTI_Line6);
	}
	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		uint32_t aChannel = Bit_SET;
		uint32_t bChannel = ReadEncoderChannelLevel(ENCODER3_B_CHANNEL);
		if (aChannel == bChannel)
		{
			//A상 인터럽트에서 aChannel == bChannel 일경우 역회전
			motorEncoders[MOTOR_ENCODER3_INDEX].encoderCount--;
			motorEncoders[MOTOR_ENCODER3_INDEX].isRotateFoward = FALSE;
		}
		else
		{
			//A상 인터럽트에서 aChannel != bChannel 일경우 정회전
			motorEncoders[MOTOR_ENCODER3_INDEX].encoderCount++;
			motorEncoders[MOTOR_ENCODER3_INDEX].isRotateFoward = TRUE;
		}

		EXTI_ClearITPendingBit(EXTI_Line7);
	}
}

void MotorEncoderRccConfig()
{
	//               B    A
	//Motor Encoder0 PA0, PA1
	//Motor Encoder1 PC2, PC3
	//Motor Encoder2 PA4, PA5
	//Motor Encoder3 PA6, PA7

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

void MotorEncoderNvicConfig()
{
	NVIC_InitTypeDef nvicExtiInit;

	//Encoder Interrupt
	nvicExtiInit.NVIC_IRQChannel = GetEncoderNvicIrqChannel(ENCODER0_A_CHANNEL);
	nvicExtiInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicExtiInit.NVIC_IRQChannelSubPriority = 1;
	nvicExtiInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicExtiInit);

	nvicExtiInit.NVIC_IRQChannel = GetEncoderNvicIrqChannel(ENCODER0_B_CHANNEL);
	nvicExtiInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicExtiInit.NVIC_IRQChannelSubPriority = 1;
	nvicExtiInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicExtiInit);

	nvicExtiInit.NVIC_IRQChannel = GetEncoderNvicIrqChannel(ENCODER1_A_CHANNEL);
	nvicExtiInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicExtiInit.NVIC_IRQChannelSubPriority = 1;
	nvicExtiInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicExtiInit);

	nvicExtiInit.NVIC_IRQChannel = GetEncoderNvicIrqChannel(ENCODER1_B_CHANNEL);
	nvicExtiInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicExtiInit.NVIC_IRQChannelSubPriority = 1;
	nvicExtiInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicExtiInit);

	nvicExtiInit.NVIC_IRQChannel = GetEncoderNvicIrqChannel(ENCODER2_A_CHANNEL);
	nvicExtiInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicExtiInit.NVIC_IRQChannelSubPriority = 1;
	nvicExtiInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicExtiInit);

	nvicExtiInit.NVIC_IRQChannel = GetEncoderNvicIrqChannel(ENCODER2_B_CHANNEL);
	nvicExtiInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicExtiInit.NVIC_IRQChannelSubPriority = 1;
	nvicExtiInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicExtiInit);

	nvicExtiInit.NVIC_IRQChannel = GetEncoderNvicIrqChannel(ENCODER3_A_CHANNEL);
	nvicExtiInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicExtiInit.NVIC_IRQChannelSubPriority = 1;
	nvicExtiInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicExtiInit);

	nvicExtiInit.NVIC_IRQChannel = GetEncoderNvicIrqChannel(ENCODER3_B_CHANNEL);
	nvicExtiInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicExtiInit.NVIC_IRQChannelSubPriority = 1;
	nvicExtiInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicExtiInit);
}

void MotorEncoderInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	//Ecnoder
	GPIO_InitStructure.GPIO_Pin = GetEncoderGpioPin(ENCODER0_A_CHANNEL);
	GPIO_Init(GetEncoderGpioPort(ENCODER0_A_CHANNEL), &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GetEncoderGpioPin(ENCODER0_B_CHANNEL);
	GPIO_Init(GetEncoderGpioPort(ENCODER0_B_CHANNEL), &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GetEncoderGpioPin(ENCODER1_A_CHANNEL);
	GPIO_Init(GetEncoderGpioPort(ENCODER1_A_CHANNEL), &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GetEncoderGpioPin(ENCODER1_B_CHANNEL);
	GPIO_Init(GetEncoderGpioPort(ENCODER1_B_CHANNEL), &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GetEncoderGpioPin(ENCODER2_A_CHANNEL);
	GPIO_Init(GetEncoderGpioPort(ENCODER2_A_CHANNEL), &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GetEncoderGpioPin(ENCODER2_B_CHANNEL);
	GPIO_Init(GetEncoderGpioPort(ENCODER2_B_CHANNEL), &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GetEncoderGpioPin(ENCODER3_A_CHANNEL);
	GPIO_Init(GetEncoderGpioPort(ENCODER3_A_CHANNEL), &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GetEncoderGpioPin(ENCODER3_B_CHANNEL);
	GPIO_Init(GetEncoderGpioPort(ENCODER3_B_CHANNEL), &GPIO_InitStructure);

	EXTI_InitTypeDef extiInit;

	//Encoder EXTI
	EXTI_ClearITPendingBit(GetEncoderExtiLine(ENCODER0_A_CHANNEL));
	EXTI_ClearITPendingBit(GetEncoderExtiLine(ENCODER0_B_CHANNEL));
	EXTI_ClearITPendingBit(GetEncoderExtiLine(ENCODER1_A_CHANNEL));
	EXTI_ClearITPendingBit(GetEncoderExtiLine(ENCODER1_B_CHANNEL));
	EXTI_ClearITPendingBit(GetEncoderExtiLine(ENCODER2_A_CHANNEL));
	EXTI_ClearITPendingBit(GetEncoderExtiLine(ENCODER2_B_CHANNEL));
	EXTI_ClearITPendingBit(GetEncoderExtiLine(ENCODER3_A_CHANNEL));
	EXTI_ClearITPendingBit(GetEncoderExtiLine(ENCODER3_B_CHANNEL));

	GPIO_EXTILineConfig(GetEncoderExtiPortSource(ENCODER0_A_CHANNEL),
			GetEncoderExtiPinSource(ENCODER0_A_CHANNEL));
	GPIO_EXTILineConfig(GetEncoderExtiPortSource(ENCODER0_B_CHANNEL),
			GetEncoderExtiPinSource(ENCODER0_B_CHANNEL));
	GPIO_EXTILineConfig(GetEncoderExtiPortSource(ENCODER1_A_CHANNEL),
			GetEncoderExtiPinSource(ENCODER1_A_CHANNEL));
	GPIO_EXTILineConfig(GetEncoderExtiPortSource(ENCODER1_B_CHANNEL),
			GetEncoderExtiPinSource(ENCODER1_B_CHANNEL));
	GPIO_EXTILineConfig(GetEncoderExtiPortSource(ENCODER2_A_CHANNEL),
			GetEncoderExtiPinSource(ENCODER2_A_CHANNEL));
	GPIO_EXTILineConfig(GetEncoderExtiPortSource(ENCODER2_B_CHANNEL),
			GetEncoderExtiPinSource(ENCODER2_B_CHANNEL));
	GPIO_EXTILineConfig(GetEncoderExtiPortSource(ENCODER3_A_CHANNEL),
			GetEncoderExtiPinSource(ENCODER3_A_CHANNEL));
	GPIO_EXTILineConfig(GetEncoderExtiPortSource(ENCODER3_B_CHANNEL),
			GetEncoderExtiPinSource(ENCODER3_B_CHANNEL));

	extiInit.EXTI_Line = GetEncoderExtiLine(ENCODER0_A_CHANNEL)
			| GetEncoderExtiLine(ENCODER0_B_CHANNEL)
			| GetEncoderExtiLine(ENCODER1_A_CHANNEL)
			| GetEncoderExtiLine(ENCODER1_B_CHANNEL)
			| GetEncoderExtiLine(ENCODER2_A_CHANNEL)
			| GetEncoderExtiLine(ENCODER2_B_CHANNEL)
			| GetEncoderExtiLine(ENCODER3_A_CHANNEL)
			| GetEncoderExtiLine(ENCODER3_B_CHANNEL);
	extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
	extiInit.EXTI_Trigger = EXTI_Trigger_Rising;
	extiInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&extiInit);
}

