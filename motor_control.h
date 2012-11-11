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

#define CONTROL_MOTOR_COUNT (2)
#define PULSE_PER_ROTATION (38 * 49)
#define DEFAULT_POSITION_ERROR_LIMIT (3.0f)
#define CONTROL_PERIOD_MS (10)

#define DEFAULT_KP (0.033f)
#define DEFAULT_KI (6.60f)
#define DEFAULT_KD (0.00011f)

#define DEFAULT_ACCELERATION (100.0f)
#define DEFAULT_MAX_VELCOITY (100.0f)

typedef enum MotorControlMode
{
	CONTROL_FREE, CONTROL_PID_TUNING, CONTROL_VELOCITY, CONTROL_POSITION
} ControlMode;

typedef struct PidParameter
{
	float kp, ki, kd;
	float error2Coeff, error1Coeff, error0Coeff;
	float errors[3]; //[0] = t-2, [1] = t-1, [2] = t

	float desiredValue;
	float currentValue;

	float mainpulatedValue;
	float manipulatedValueLimit;
	float dt;
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

void SetKp(MotorControlType* pMotorControlType, float kp_);
void SetKi(MotorControlType* pMotorControlType, float ki_);
void SetKd(MotorControlType* pMotorControlType, float kd_);

extern MotorControlType motorControlTypes[CONTROL_MOTOR_COUNT];

#endif /* MOTOR_CONTROL_TIMER_H_ */
