/*
 * motor_control_timer.c
 *
 *  Created on: 2012. 8. 7.
 *      Author: East
 */
#include "motor_control.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "CMSIS/misc.h"
#include "CMSIS/stm32f10x_rcc.h"
#include "CMSIS/stm32f10x_tim.h"
#include "CMSIS/stm32f10x_gpio.h"

#include "system_timer.h"
#include "motor_encoder.h"
#include "i2c.h"

#define NT_S_DCDM1210_0_I2C_ADRESS 0x07
#define NT_S_DCDM1210_1_I2C_ADRESS 0x08

#define BOUNDARY(_X_, _MIN_, _MAX_) (_X_ < _MIN_ ? _MIN_ : _X_ > _MAX_ ? _MAX_ : _X_)
#define INTEGER(_X_) (_X_ > 0 ? (int)(_X_ + 0.5) : (int)(_X_ - 0.5))

float CalculateManipulatedValue(PidParameter* const pid);
void SetMotorPwmTo(uint16_t id, float motor0, float motor1);

float CacluateAverage(float* values, unsigned int size);
float CalculateMovingAverage(MovingAverageType* movingAverageType, float value);

int __errno;

static volatile BOOL isOnTimeForMotorControlProcess = FALSE;

MotorControlType motorControlTypes[CONTROL_MOTOR_COUNT] =
{
{ CONTROL_FREE, 0.0f,
{ 0.0f, 0.0f, 0.033f, 6.60f, 0.00011f, 0.0f, 0.0f, 0.0f, 0.0f, 999.0f,
		CONTROL_PERIOD_MS / 1000.0f, 0.0f, 0.0f },
{ 100.0f, 100.0f },
{
{ 0.0f, }, 0 }, 0.0f, DEFAULT_POSITION_ERROR_LIMIT, PULSE_PER_ROTATION, 0, 0, 0,
		0.0f },
{ CONTROL_FREE, 0.0f,
{ 0.0f, 0.0f, 0.033f, 6.60f, 0.00011f, 0.0f, 0.0f, 0.0f, 0.0f, 999.0f,
		CONTROL_PERIOD_MS / 1000.0f, 0.0f, 0.0f },
{ 100.0f, 100.0f },
{
{ 0.0f, }, 0 }, 0.0f, DEFAULT_POSITION_ERROR_LIMIT, PULSE_PER_ROTATION, 0, 0, 0,
		0.0f } };

void TIM2_IRQHandler()
{
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	isOnTimeForMotorControlProcess = TRUE;
}

void MotorControlProcess()
{
	if (isOnTimeForMotorControlProcess == FALSE)
		return;
	else
		isOnTimeForMotorControlProcess = FALSE;

	float manipulatedValues[CONTROL_MOTOR_COUNT] =
	{ 0.0f, 0.0f };
	int64_t currentEcnoderValues[CONTROL_MOTOR_COUNT] =
	{ motorEncoders[0].encoderCount, motorEncoders[1].encoderCount };

	int i;
	for (i = 0; i < CONTROL_MOTOR_COUNT; i++)
	{
		const float controlDesiredValue =
				motorControlTypes[i].controlDeisredValue;
		const float dt = motorControlTypes[i].pid.dt;
		const float acceleration =
				motorControlTypes[i].velocityParameter.acceleration;
		const float maxVelocity =
				motorControlTypes[i].velocityParameter.maximumVelocity;
		const float velocityUnit = acceleration * dt;
		const float pulsePerRotation = motorControlTypes[i].pulsePerRotation;
		const float currentVelocity = (currentEcnoderValues[i]
				- motorControlTypes[i].previousEncoder) * 360.0f
				/ (pulsePerRotation * dt); //degree/s
		const float averageVelocity = CalculateMovingAverage(
				&motorControlTypes[i].movingAverageVelocitySamples,
				currentVelocity); //degree/s
		const uint32_t time = motorControlTypes[i].time;

		switch (motorControlTypes[i].controlMode)
		{
		case CONTROL_VELOCITY:
		{
			float startVelocity = motorControlTypes[i].startVelocity;
			uint32_t desiredTime = motorControlTypes[i].desiredTime;

			if (time == 0)
			{
				startVelocity = averageVelocity;
				desiredTime = fabsf(controlDesiredValue - startVelocity)
						/ acceleration / dt;

				motorControlTypes[i].startVelocity = startVelocity;
				motorControlTypes[i].desiredTime = desiredTime;
			}

			float acceledDesiredVelocity = 0.0f;
			if (time < desiredTime)
			{
				acceledDesiredVelocity = startVelocity
						+ (controlDesiredValue - startVelocity > 0 ? 1.0 : -1.0)
								* velocityUnit * time;
			}
			else
			{
				acceledDesiredVelocity = controlDesiredValue;
			}

			motorControlTypes[i].pid.desiredValue = acceledDesiredVelocity;
			motorControlTypes[i].pid.currentValue = averageVelocity;
			manipulatedValues[i] = CalculateManipulatedValue(
					&motorControlTypes[i].pid);
		}
			break;
		case CONTROL_POSITION:
		{
			const float currentPosition = currentEcnoderValues[i] * 360.0f
					/ pulsePerRotation;

			const float positionError = controlDesiredValue - currentPosition;
			const float absOfPositionError = fabsf(positionError);
			const float positionErrorSign = positionError > 0.0f ? 1.0f : -1.0f;

			uint32_t desiredTime = motorControlTypes[i].desiredTime;
			float acceledDesiredVelocity = 0.0f;
			if (time > desiredTime)
			{
				acceledDesiredVelocity =
						absOfPositionError
								< motorControlTypes[i].positionErrorLimit ?
								0.0 :
								positionErrorSign
										* sqrtf(
												2 * acceleration
														* absOfPositionError);
				acceledDesiredVelocity =
						BOUNDARY(acceledDesiredVelocity, -maxVelocity, maxVelocity);
			}
			else
			{
				float startVelocity = motorControlTypes[i].startVelocity;

				if (time == 0)
				{
					const float desiredVelocity = positionErrorSign
							* maxVelocity;

					startVelocity =
							BOUNDARY(averageVelocity, -maxVelocity, maxVelocity);

					uint32_t startAreaDistance = fabsf(
							(desiredVelocity - startVelocity)
									* (desiredVelocity + startVelocity)
									/ (2 * acceleration));
					uint32_t endAreaDistance = desiredVelocity * desiredVelocity
							/ (2 * acceleration);

					if (absOfPositionError
							< startAreaDistance + endAreaDistance)
					{
						float desiredFloatTime =
								fabsf(
										(-2 * startVelocity
												+ sqrtf(
														2 * startVelocity
																* startVelocity
																+ 4
																		* acceleration
																		* absOfPositionError))
												/ (2 * acceleration) / dt);
						desiredTime = INTEGER(desiredFloatTime);
					}
					else
					{
						float desiredFloatTime = fabsf(
								desiredVelocity - startVelocity) / acceleration
								/ dt;
						desiredTime = INTEGER(desiredFloatTime);
					}

					motorControlTypes[i].startVelocity = startVelocity;
					motorControlTypes[i].desiredTime = desiredTime;
				}

				acceledDesiredVelocity = startVelocity
						+ positionErrorSign * velocityUnit * time;
			}

			motorControlTypes[i].pid.desiredValue = acceledDesiredVelocity;
			motorControlTypes[i].pid.currentValue = averageVelocity;
			manipulatedValues[i] = CalculateManipulatedValue(
					&motorControlTypes[i].pid);
		}
			break;
		case CONTROL_PID_TUNING:
		{
			motorControlTypes[i].pid.desiredValue = controlDesiredValue;
			motorControlTypes[i].pid.currentValue = currentVelocity;
			manipulatedValues[i] = CalculateManipulatedValue(
					&motorControlTypes[i].pid);
		}
			break;
		default:
			manipulatedValues[i] = 0.0f;
			break;
		}

		if (motorControlTypes[i].time != 0xFFFFFFFF)
			motorControlTypes[i].time += 1;
		motorControlTypes[i].previousEncoder = currentEcnoderValues[i];
		motorControlTypes[i].averageVelocity = averageVelocity;
	}

	//manipulatedValues[0] = -100;

	SetMotorPwmTo(NT_S_DCDM1210_0_I2C_ADRESS, manipulatedValues[0],
			manipulatedValues[1]);
	//SetMotorPwmTo(0x08, manipulatedValues[2], manipulatedValues[3]);
}

float CalculateManipulatedValue(PidParameter* const pid)
{
	const float error = pid->desiredValue - pid->currentValue;
	const float dt = pid->dt;
	const float maxLimitValue = pid->manipulatedValueLimit;
	const float minLimitValue = -maxLimitValue;

	float proportionalTerm = pid->proportionalGain * error;
	float integralTerm = pid->integralTerm + pid->integralGain * error * dt;
	float derivativeTerm = pid->derivativeGain * (error - pid->previousError)
			/ dt;

	//proportionalTerm = BOUNDARY(proportionalTerm, minLimitValue, maxLimitValue);
	integralTerm = BOUNDARY(integralTerm, minLimitValue, maxLimitValue);
	//derivativeTerm = BOUNDARY(derivativeTerm, minLimitValue, maxLimitValue);

	float manipulatedValue = proportionalTerm + integralTerm + derivativeTerm;
	manipulatedValue = BOUNDARY(manipulatedValue, minLimitValue, maxLimitValue);

	//for debuging//
	pid->error = error;
	pid->proportionalTerm = proportionalTerm;
	pid->derivativeTerm = derivativeTerm;
	pid->mainpulatedValue = manipulatedValue;
	////////////////

	pid->integralTerm = integralTerm;
	pid->previousError = error;

	return manipulatedValue;
}

void SetMotorPwmTo(uint16_t id, float motor0, float motor1)
{
	int motor0Pwm = INTEGER(motor0);
	int motor1Pwm = INTEGER(motor1);

	motor0Pwm = BOUNDARY(motor0Pwm, -999, 999);
	motor1Pwm = BOUNDARY(motor1Pwm, -999, 999);

	motor0Pwm = abs(motor0Pwm);
	motor1Pwm = abs(motor1Pwm);

	static uint8_t message[13];

	message[0] = '<';
	message[1] = '0';
	message[2] = 'R';
	message[3] = motor0 > 0 ? '1' : '0';
	message[4] = (motor0Pwm / 100) % 10 + '0';
	message[5] = (motor0Pwm / 10) % 10 + '0';
	message[6] = (motor0Pwm) % 10 + '0';
	message[7] = 'L';
	message[8] = motor1 > 0 ? '1' : '0';
	message[9] = (motor1Pwm / 100) % 10 + '0';
	message[10] = (motor1Pwm / 10) % 10 + '0';
	message[11] = (motor1Pwm) % 10 + '0';
	message[12] = '>';

	if (IsI2c2OnGoing() == TRUE)
		return;
	I2c2WriteRequest(id, message, sizeof(message));
}

float CacluateAverage(float* values, unsigned int size)
{
	float average = 0.0f;
	int i;
	for (i = 0; i < size; i++)
	{
		average += values[i];
	}
	average /= size;

	return average;
}

float CalculateMovingAverage(MovingAverageType* movingAverageType, float value)
{
	const int size = sizeof(movingAverageType->values)
			/ sizeof(movingAverageType->values[0]);
	int index = movingAverageType->index;

	if (index >= size)
		index = 0;

	movingAverageType->values[index] = value;
	movingAverageType->index = index + 1;

	return CacluateAverage(movingAverageType->values, size);
}

void MotorControlTimerRccConfig()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void MotorControlTimerNvicConfig()
{
	//Timer Interrupt
	NVIC_InitTypeDef nvicTiemrInit;
	nvicTiemrInit.NVIC_IRQChannel = TIM2_IRQn;
	nvicTiemrInit.NVIC_IRQChannelPreemptionPriority = 1;
	nvicTiemrInit.NVIC_IRQChannelSubPriority = 0;
	nvicTiemrInit.NVIC_IRQChannelCmd = ENABLE;

	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

	NVIC_Init(&nvicTiemrInit);
}

void MotorControlTimerInit()
{
	TIM_TimeBaseInitTypeDef timeBaseInit;

	timeBaseInit.TIM_Period = CONTROL_PERIOD_MS * 10;
	timeBaseInit.TIM_Prescaler = 7200;
	timeBaseInit.TIM_ClockDivision = 0;
	timeBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	timeBaseInit.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &timeBaseInit);

	TIM_ClearFlag(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

