/*
 * motor_encoder.h
 *
 *  Created on: 2012. 8. 6.
 *      Author: East
 */

#ifndef MOTOR_ENCODER_H_
#define MOTOR_ENCODER_H_

#include <inttypes.h>
#include "type.h"

#define MOTOR_ENCODER0_INDEX 0
#define MOTOR_ENCODER1_INDEX 1
#define MOTOR_ENCODER2_INDEX 2
#define MOTOR_ENCODER3_INDEX 3
#define MOTOR_ENCODER_COUNT 4

typedef struct MotorEncoder
{
	volatile int64_t encoderCount;
	volatile BOOL isRotateFoward;
} MotorEncoder;

void MotorEncoderRccConfig();
void MotorEncoderNvicConfig();
void MotorEncoderInit();

extern MotorEncoder motorEncoders[MOTOR_ENCODER_COUNT];

#endif /* MOTOR_ENCODER_H_ */
