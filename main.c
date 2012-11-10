/*
 * main.c
 *
 *  Created on: 2012. 4. 12.
 *      Author: East
 */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "CMSIS/stm32f10x.h"
#include "CMSIS/stm32f10x_it.h"
#include "CMSIS/misc.h"
#include "CMSIS/stm32f10x_usart.h"
#include "CMSIS/stm32f10x_i2c.h"
#include "CMSIS/stm32f10x_rcc.h"
#include "CMSIS/stm32f10x_gpio.h"
#include "CMSIS/stm32f10x_exti.h"
#include "CMSIS/stm32f10x_tim.h"

#include "USB/hw_config.h"
#include "USB/Interface/usb_istr.h"
#include "USB/Driver/usb_core.h"
#include "USB/Driver/usb_init.h"
#include "USB/Interface/usb_pwr.h"

#include "motor_encoder.h"
#include "motor_control.h"
#include "i2c.h"
#include "system_timer.h"

#define I2C_CLOCK_SPEED             300000
#define NT_S_DCDM1210_0_I2C_ADRESS 0x07

volatile BOOL hasRebootRequest = TRUE;
volatile BOOL hasSystemError = FALSE;

void UsartRccConfig();
void UsartNvicConfig();
void UsartInit(uint32_t baudrate);

void LedRccConfig();
void LedInit();

void Reboot();

void ApplyUpdatedMotionControllerCommand();

int main()
{
	SystemInit();

#ifndef DEBUG
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x2000);
#endif
	SystemTimerInit();

	LedRccConfig();
	UsartRccConfig();
	I2cRccConfig();
	MotorEncoderRccConfig();
	MotorControlTimerRccConfig();
	Set_USBClock();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 선점형 인터럽트
	UsartNvicConfig();
	I2cNvicConfig();
	MotorEncoderNvicConfig();
	MotorControlTimerNvicConfig();
	USB_Interrupts_Config();

	LedInit();
	UsartInit(230400);
	I2cInit(I2C_CLOCK_SPEED);
	MotorEncoderInit();
	USB_GPIO_Configuration();

	USB_Init();

	GPIO_ResetBits(GPIOC, GPIO_Pin_8); // LED on

	Delay_ms(3000); //Waiting for NT-S-DCDM1210
	int i;

	char* accelerationMotorDriveCommand = "<0E000>"; //가감속 설정
	I2c2WriteRequest(NT_S_DCDM1210_0_I2C_ADRESS,
			(unsigned char*) accelerationMotorDriveCommand,
			strlen(accelerationMotorDriveCommand));
	for (i = 0; IsI2c2OnGoing() && i < 10; i++)
	{
		Delay_ms(10);
	}
	assert_param(IsI2c2Error() == FALSE && "Setting Acceleration Error");

	char* deadbandMotorDriveCommand = "<0D000>"; //데드밴드
	I2c2WriteRequest(NT_S_DCDM1210_0_I2C_ADRESS,
			(unsigned char*) deadbandMotorDriveCommand,
			strlen(deadbandMotorDriveCommand));
	for (i = 0; IsI2c2OnGoing() && i < 10; i++)
	{
		Delay_ms(10);
	}
	assert_param(IsI2c2Error() == FALSE && "Setting Dead Band Error");

	char* stopMotorDriveCommand = "<0R0000L0000>"; //정지
	I2c2WriteRequest(NT_S_DCDM1210_0_I2C_ADRESS,
			(unsigned char*) stopMotorDriveCommand,
			strlen(stopMotorDriveCommand));
	for (i = 0; IsI2c2OnGoing() && i < 10; i++)
	{
		Delay_ms(10);
	}
	assert_param(IsI2c2Error() == FALSE && "Stop Error");

	///////////////////////////////////////////////////////////
	MotorControlTimerInit();

	hasRebootRequest = FALSE;

	uint32_t tickCount = GetSystemTickCount();
	for (;;)
	{
		if (hasRebootRequest == TRUE)
		{
			for (i = 0; i < 100 && IsI2c2OnGoing() == TRUE; i++)
			{
				GPIOC->ODR ^= GPIO_Pin_8; // LED Blink
				Delay_ms(10);
			}

			Reboot();
		}
		else
		{
			ApplyUpdatedMotionControllerCommand();
			MotorControlProcess();
		}

		if (TimeInterval(tickCount) >= 100000)
		{
			tickCount = GetSystemTickCount();
			GPIOC->ODR ^= GPIO_Pin_8; // LED Blink
		}
	}

	return 0;
}

void USART1_IRQHandler()
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		uint16_t data = USART_ReceiveData(USART1);
		if (data == 27)
		{
			printf("Reboot \r\n");
			Reboot();
		}
	}
}

void USB_LP_CAN1_RX0_IRQHandler()
{
	USB_Istr();
}

void assert_failed(char* message, uint8_t* file, uint32_t line)
{
	hasSystemError = TRUE;
	uint32_t count = 0;

	char* resetMotorDriveCommand = "<0r>"; //정지
	I2c2WriteRequest(NT_S_DCDM1210_0_I2C_ADRESS,
			(unsigned char*) resetMotorDriveCommand,
			strlen(resetMotorDriveCommand));

	for (;;)
	{
		printf("%lu // %s : %s(%lu)\r\n", ++count, message, (char*) file, line);
		GPIOC->ODR ^= GPIO_Pin_8; // LED Blink
		Delay_ms(3000);
	}
}

void Reboot()
{
	PowerOff();

	char* resetMotorDriveCommand = "<0r>"; //정지
	I2c2WriteRequest(NT_S_DCDM1210_0_I2C_ADRESS,
			(unsigned char*) resetMotorDriveCommand,
			strlen(resetMotorDriveCommand));

	int i;
	for (i = 0; i < 10 && IsI2c2OnGoing() == TRUE; i++)
	{
		Delay_ms(10);
	}

	NVIC_SystemReset();
}

void UsartRccConfig(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void LedRccConfig()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

void UsartNvicConfig()
{
	//USART Interrupt
	NVIC_InitTypeDef nvicUsartInit;
	nvicUsartInit.NVIC_IRQChannel = USART1_IRQn;
	nvicUsartInit.NVIC_IRQChannelPreemptionPriority = 2;
	nvicUsartInit.NVIC_IRQChannelSubPriority = 0;
	nvicUsartInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicUsartInit);
}

void UsartInit(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USARTx_Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USARTx_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void LedInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void _init()
{
}
