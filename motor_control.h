/*
 * motor_control_timer.h
 *
 *  Created on: 2012. 8. 7.
 *      Author: East
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <inttypes.h>
#include "type.h"

#define CONTROL_MOTOR_COUNT 2
#define PULSE_PER_ROTATION (38 * 49)
#define DEFAULT_POSITION_ERROR_LIMIT (3)
#define CONTROL_PERIOD_MS 10

typedef enum MotorControlMode
{
	CONTROL_FREE, CONTROL_PID_TUNING, CONTROL_VELOCITY, CONTROL_POSITION
} ControlMode;

typedef struct PidParameter
{
	float desiredValue;
	float currentValue;

	float proportionalGain;
	float integralGain;
	float derivativeGain;

	//for debuging
	float error, proportionalTerm, derivativeTerm, mainpulatedValue;

	//
	float manipulatedValueLimit;
	float dt;

	//read-only
	float integralTerm;
	float previousError;
} PidParameter;

typedef struct MovingAverageType
{
	float values[5];
	unsigned int index;
} MovingAverageType;

typedef struct VelocityParameter
{
	float acceleration;
	float maximumVelocity;
} VelocityParameter;

typedef struct MotorControlType
{
	ControlMode controlMode;
	float controlDeisredValue;

	PidParameter pid;
	VelocityParameter velocityParameter;

	MovingAverageType movingAverageVelocitySamples;
	float averageVelocity;
	float positionErrorLimit;

	float pulsePerRotation;
	int64_t previousEncoder;
	uint32_t time;
	uint32_t desiredTime;
	float startVelocity;
} MotorControlType;

void MotorControlTimerRccConfig();
void MotorControlTimerNvicConfig();
void MotorControlTimerInit();

void MotorControlProcess();

extern MotorControlType motorControlTypes[CONTROL_MOTOR_COUNT];

#endif /* MOTOR_CONTROL_TIMER_H_ */
