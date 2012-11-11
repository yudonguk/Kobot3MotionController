/**
 ******************************************************************************
 * @file    usb_endp.c
 * @author  MCD Application Team
 * @version V3.4.0
 * @date    29-June-2012
 * @brief   Endpoint routines
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

//#include "hw_config.h"
#include "stm32f10x_gpio.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "../../system_timer.h"

#include <stdio.h>

#include "../../type.h"
#include "../../motor_encoder.h"
#include "../../motor_control.h"

typedef struct MotionControllerCommand
{
	uint8_t command;
	uint8_t option;

	union
	{
		uint8_t rawData[4];
		int32_t int32Data;
		uint32_t uint32Data;
		float float32Data;
		uint32_t errorCode;
	} data;
} MotionControllerCommand;

enum MotionControllerErrorCode
{
	None = 0,
	UnknownError = 1,
	NotSupportedCommand = 2,
	NotHasProcessedPreviousCommand = 3,
	InvalidMotorId = 10,
	EnteringRebootingProcess = 11
};

enum MotionControllerStopMode
{
	EmergencyStop = 0, DeaccelerationStop = 1, FreeWheeling = 2
};

#define MAX_MOTION_CONTROLLER_COMMAND_COUNT 4
#define MOTION_CONTROLLER_HID_REPORT_ID_SIZE 1
#define MOTION_CONTROLLER_COMMAND_SIZE 6

#define USB_TX_RX_BUFFER_SIZE 128

int ParseMotionControllerCommandHidPacket(
		MotionControllerCommand commands[MAX_MOTION_CONTROLLER_COMMAND_COUNT],
		uint8_t* packetData, uint32_t packetDataSize);
int ConvertMotionControllerCommandToHidPacket(uint8_t* packetData,
		uint32_t packetDataAvailableSize,
		MotionControllerCommand commands[MAX_MOTION_CONTROLLER_COMMAND_COUNT],
		uint32_t commandCount);
void ProcessMotionControllerCommand(
		MotionControllerCommand commands[MAX_MOTION_CONTROLLER_COMMAND_COUNT],
		uint32_t commandCount);

/////////////////////////////////////////////////////////////////////////////
MotionControllerCommand updatedCommand[MAX_MOTION_CONTROLLER_COMMAND_COUNT];
uint32_t updatedCommnadCount = 0;

volatile enum
{
	NO_COMMAND_UPDATE = 0, COMMAND_UPDATED = 1, PROCESSING_COMMAND_UPDATE = 3
} processedMotionControllerCommand = NO_COMMAND_UPDATE;

static uint8_t usbTxRxBuffer[USB_TX_RX_BUFFER_SIZE];
static MotionControllerCommand commandList[MAX_MOTION_CONTROLLER_COMMAND_COUNT];
static uint32_t systemTickCount = 0;
extern volatile BOOL hasRebootRequest;
extern volatile BOOL hasSystemError;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * Function Name  : EP1_OUT_Callback.
 * Description    : EP1 OUT Callback Routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void EP1_OUT_Callback(void)
{
	systemTickCount = GetSystemTickCount();
	/* Read received data */
	uint32_t receivedSize = USB_SIL_Read(EP1_OUT, usbTxRxBuffer);

	int commandCount = ParseMotionControllerCommandHidPacket(commandList,
			usbTxRxBuffer, receivedSize);

	if (commandCount < 0)
	{
		printf("Received Invalid USB Data\r\n");
	}
	else
	{
		ProcessMotionControllerCommand(commandList, commandCount);

		int sendPacketSize = ConvertMotionControllerCommandToHidPacket(
				usbTxRxBuffer, USB_TX_RX_BUFFER_SIZE, commandList,
				commandCount);

		if (sendPacketSize < 0)
		{
			printf("Can't convert Response to USB Data\r\n");
		}
		else
		{
			USB_SIL_Write(EP1_IN, usbTxRxBuffer, sendPacketSize);
			SetEPTxValid(ENDP1 );
		}
	}

#ifndef STM32F10X_CL   
	SetEPRxStatus(ENDP1, EP_RX_VALID);
#endif /* STM32F10X_CL */
}

/*******************************************************************************
 * Function Name  : EP1_IN_Callback.
 * Description    : EP1 IN Callback Routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void EP1_IN_Callback(void)
{
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

void ProcessMotionControllerCommand(
		MotionControllerCommand commands[MAX_MOTION_CONTROLLER_COMMAND_COUNT],
		uint32_t commandCount)
{
	//commands 는 입력 및 출력 파라미터 이다.
	int i;

	//encoder, velocity, position(Encoder 로 추론 가능) 은 시간 동기화를 위해 미리 읽어 온다.
	int64_t encoders[CONTROL_MOTOR_COUNT] =
	{ motorEncoders[0].encoderCount, motorEncoders[1].encoderCount };
	float velocitys[CONTROL_MOTOR_COUNT] =
			{ motorControlTypes[0].averageVelocity,
					motorControlTypes[1].averageVelocity };

	BOOL hasProcesseCommand = FALSE;

	if (processedMotionControllerCommand != PROCESSING_COMMAND_UPDATE)
	{
		updatedCommnadCount = 0;
	}

	for (i = 0; i < commandCount; i++)
	{
		//선행 상태확인
		if (hasRebootRequest == TRUE)
		{
			commands[i].option |= 0b10000000;
			commands[i].data.errorCode = EnteringRebootingProcess;
			continue;
		}
		else if (hasSystemError == TRUE)
		{
			commands[i].option |= 0b10000000;
			commands[i].data.errorCode = UnknownError;
			continue;
		}

		switch (commands[i].command)
		{
		//아래 명령들은 이전 제어 명령이 완료 되었건 되지 않았건
		//동작해도 되는 명령이기에
		//isProcessedMotionControllerCommand 의 값 에 상관없이
		//수행될 수 있다.
		case 0:
			//Reset
			hasRebootRequest = TRUE;
			break;

			/////////////////////////////////////////////////////////////////
			//motor control
			//아래 명령들은 이전 제어 명령이 완료 된후 동작 해야한다.
			//따라서 isProcessedMotionControllerCommand 의 값이 TRUE일때 수행된다.
		case 1:
			//PID Tuning
		case 2:
			//Control Velocity
		case 3:
			//Control Position
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else if (processedMotionControllerCommand
					== PROCESSING_COMMAND_UPDATE)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = NotHasProcessedPreviousCommand; // 아직 이전 명령이 수행 되지 않음
			}
			else
			{
				updatedCommand[updatedCommnadCount++] = commands[i];
				hasProcesseCommand = TRUE;
			}
			break;

		case 4:
			//Stop
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else if (processedMotionControllerCommand
					== PROCESSING_COMMAND_UPDATE)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = NotHasProcessedPreviousCommand; // 아직 이전 명령이 수행 되지 않음
			}
			else
			{
				switch (commands[i].data.uint32Data)
				{
				case EmergencyStop:
				case DeaccelerationStop:
				case FreeWheeling:
					updatedCommand[updatedCommnadCount++] = commands[i];
					hasProcesseCommand = TRUE;
					break;

				default:
					//에러
					commands[i].option |= 0b10000000;
					commands[i].data.errorCode = 2; // 지원되지 않는 명령
					break;
				}
			}
			break;

			/////////////////////////////////////////////////////////////////
			//get motor status
			//아래 명령들은 이전 제어 명령이 완료 되었건 되지 않았건
			//동작해도 되는 명령이기에
			//isProcessedMotionControllerCommand 의 값 에 상관없이
			//수행될 수 있다.
		case 9:
			//Get Status
			commands[i].data.errorCode = None;
			break;

		case 10:
			//Get Encoder
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.int32Data = encoders[commands[i].option];
			}
			break;

		case 11:
			//Get Velocity
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.int32Data = velocitys[commands[i].option]
						/ 360.0 * 1000;
			}
			break;

		case 12:
			//Get Position
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				const uint32_t motorId = commands[i].option;
				commands[i].data.int32Data = encoders[motorId] * 1000
						/ motorControlTypes[motorId].pulsePerRotation;
			}
			break;

			/////////////////////////////////////////////////////////////////
			//get motor control parameter
			//아래 명령들은 이전 제어 명령이 완료 되었건 되지 않았건
			//동작해도 되는 명령이기에
			//isProcessedMotionControllerCommand 의 값 에 상관없이
			//수행될 수 있다.
		case 20:
			//Get PPR
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.float32Data =
						motorControlTypes[commands[i].option].pulsePerRotation;
			}
			break;

		case 21:
			//Get Acceleration
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.int32Data =
						motorControlTypes[commands[i].option].velocityParameter.acceleration
								/ 360.0 * 1000;
			}
			break;

		case 22:
			//Get Max Velocity for Control Positon
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.int32Data =
						motorControlTypes[commands[i].option].velocityParameter.maximumVelocity
								/ 360.0 * 1000;
			}
			break;

		case 23:
			//Get Position Error Limit
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.int32Data =
						motorControlTypes[commands[i].option].positionErrorLimit
								/ 360.0 * 1000;
			}
			break;

		case 24:
			//Get P gain
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.float32Data =
						motorControlTypes[commands[i].option].pid.kp;
			}
			break;

		case 25:
			//Get I gain
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.float32Data =
						motorControlTypes[commands[i].option].pid.ki;
			}
			break;

		case 26:
			//Get D gain
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else
			{
				commands[i].data.float32Data =
						motorControlTypes[commands[i].option].pid.kd;
			}
			break;

			/////////////////////////////////////////////////////////////////
			//set motor control paramter
			//아래 명령들은 이전 제어 명령이 완료 된후 동작 해야한다.
			//따라서 isProcessedMotionControllerCommand 의 값이 TRUE일때 수행된다.
		case 148:
			//Set PPR
		case 149:
			//Set Acceleration
		case 150:
			//Set Max Velocity for control Position
		case 151:
			//Set Position Error Limit
		case 152:
			//Set P gain
		case 153:
			//Set I gain
		case 154:
			//Set D gain
			if (commands[i].option >= CONTROL_MOTOR_COUNT)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = InvalidMotorId; // 지원하지 않는 모터 아이디
			}
			else if (processedMotionControllerCommand
					== PROCESSING_COMMAND_UPDATE)
			{
				commands[i].option |= 0b10000000;
				commands[i].data.errorCode = 1; // 아직 이전 명령이 수행 되지 않음
			}
			else
			{
				updatedCommand[updatedCommnadCount++] = commands[i];
				hasProcesseCommand = TRUE;
			}
			break;

			/////////////////////////////////////////////////////////////////
		default:
			commands[i].option |= 0b10000000;
			commands[i].data.errorCode = NotSupportedCommand; // 지원되지 않는 명령
			break;
		}
	}

	if (hasProcesseCommand == TRUE)
		processedMotionControllerCommand = COMMAND_UPDATED;
}

void ApplyUpdatedMotionControllerCommand()
{
	if (processedMotionControllerCommand == NO_COMMAND_UPDATE)
		return;

	processedMotionControllerCommand = PROCESSING_COMMAND_UPDATE;

	int64_t encoders[CONTROL_MOTOR_COUNT] =
	{ motorEncoders[0].encoderCount, motorEncoders[1].encoderCount };

	//읽기 작없외 쓰기작업이 이 함수를 통해 이루어진다.
	//되도록 이 함수는 MotorControlProcess() 전에
	//호출해야한다.
	int i;
	for (i = 0; i < updatedCommnadCount; i++)
	{
		switch (updatedCommand[i].command)
		{
		/////////////////////////////////////////////////////////////////
		//motor control
		//아래 명령들은 이전 제어 명령이 완료 된후 동작 해야한다.
		//따라서 isProcessedMotionControllerCommand 의 값이 TRUE일때 수행된다.
		case 1:
			//PID Tuning
		case 2:
			//Control Velocity
		case 3:
			//Control Position
		{
			const uint32_t motorId = updatedCommand[i].option;

			switch (updatedCommand[i].command)
			{
			case 1:
				motorControlTypes[motorId].controlMode = CONTROL_PID_TUNING;
				motorControlTypes[motorId].controlDeisredValue =
						updatedCommand[i].data.int32Data * 360.0 / 1000;
				break;

			case 2:
				motorControlTypes[motorId].controlMode = CONTROL_VELOCITY;
				motorControlTypes[motorId].controlDeisredValue =
						updatedCommand[i].data.int32Data * 360.0 / 1000;
				break;

			case 3:
				motorControlTypes[motorId].controlMode = CONTROL_POSITION;
				motorControlTypes[motorId].controlDeisredValue =
						(encoders[motorId]
								/ motorControlTypes[motorId].pulsePerRotation
								+ updatedCommand[i].data.int32Data / 1000.0)
								* 360.0;
				break;
			}

			motorControlTypes[motorId].time = 0;
		}
			break;

		case 4:
			//Stop
		{
			const uint32_t motorId = updatedCommand[i].option;

			switch (updatedCommand[i].data.uint32Data)
			{
			case EmergencyStop:
				motorControlTypes[motorId].controlMode = CONTROL_VELOCITY;
				motorControlTypes[motorId].controlDeisredValue = 0;
				//time 이 desiredTime 을 넘어서면
				//가속도 제어는 하지 않고 PID 제어 만하게 되어
				//바로 속도 0이 될 수 있음
				motorControlTypes[motorId].time = 0xFFFFFFFF;
				break;

			case DeaccelerationStop:
				motorControlTypes[motorId].controlMode = CONTROL_VELOCITY;
				motorControlTypes[motorId].controlDeisredValue = 0;
				motorControlTypes[motorId].time = 0;
				break;

			case FreeWheeling:
				motorControlTypes[motorId].controlMode = CONTROL_FREE;
				motorControlTypes[motorId].controlDeisredValue = 0;
				motorControlTypes[motorId].time = 0;
				break;
			}
		}
			break;

			/////////////////////////////////////////////////////////////////
			//set motor control paramter
			//아래 명령들은 이전 제어 명령이 완료 된후 동작 해야한다.
			//따라서 isProcessedMotionControllerCommand 의 값이 TRUE일때 수행된다.
		case 148:
			//Set PPR
		{
			motorControlTypes[updatedCommand[i].option].pulsePerRotation =
					updatedCommand[i].data.float32Data;
		}
			break;

		case 149:
			//Set Acceleration
		{
			motorControlTypes[updatedCommand[i].option].velocityParameter.acceleration =
					updatedCommand[i].data.int32Data * 360.0 / 1000;
		}
			break;

		case 150:
			//Set Max Velocity for control Position
		{
			motorControlTypes[updatedCommand[i].option].velocityParameter.maximumVelocity =
					updatedCommand[i].data.int32Data * 360.0 / 1000;
		}
			break;

		case 151:
			//Set Position Error Limit
		{
			motorControlTypes[updatedCommand[i].option].positionErrorLimit =
					updatedCommand[i].data.int32Data * 360.0 / 1000;
		}
			break;

		case 152:
			//Set P gain
		{
			SetKp(&motorControlTypes[updatedCommand[i].option],
					updatedCommand[i].data.float32Data);
		}
			break;

		case 153:
			//Set I gain
		{
			SetKi(&motorControlTypes[updatedCommand[i].option],
					updatedCommand[i].data.float32Data);
		}
			break;

		case 154:
			//Set D gain
		{
			SetKd(&motorControlTypes[updatedCommand[i].option],
					updatedCommand[i].data.float32Data);
		}
			break;

		}
	}

	processedMotionControllerCommand = NO_COMMAND_UPDATE; //명령 복사 완료
}

int ParseMotionControllerCommandHidPacket(
		MotionControllerCommand commands[MAX_MOTION_CONTROLLER_COMMAND_COUNT],
		uint8_t* packetData, uint32_t packetDataSize)
{
	int commandCount = packetData[0];

	if (commandCount * MOTION_CONTROLLER_COMMAND_SIZE
			+ MOTION_CONTROLLER_HID_REPORT_ID_SIZE > packetDataSize)
	{
		return -1;
	}
	else if (commandCount > MAX_MOTION_CONTROLLER_COMMAND_COUNT)
	{
		return -2;
	}

	int i;
	for (i = 0; i < commandCount; i++)
	{
		uint8_t* commandPointer = packetData
				+ MOTION_CONTROLLER_HID_REPORT_ID_SIZE
				+ i * MOTION_CONTROLLER_COMMAND_SIZE;

		commands[i].command = *commandPointer;
		commands[i].option = *(commandPointer + 1);
		// rawData[4] 를 한번에 복사하기 위함
		commands[i].data.uint32Data = *((uint32_t*) (commandPointer + 2));
	}

	return commandCount;
}

int ConvertMotionControllerCommandToHidPacket(uint8_t* packetData,
		uint32_t packetDataAvailableSize,
		MotionControllerCommand commands[MAX_MOTION_CONTROLLER_COMMAND_COUNT],
		uint32_t commandCount)
{
	int packetSize = commandCount * MOTION_CONTROLLER_COMMAND_SIZE
			+ MOTION_CONTROLLER_HID_REPORT_ID_SIZE;

	if (packetSize > packetDataAvailableSize)
	{
		return -1;
	}
	else if (commandCount > MAX_MOTION_CONTROLLER_COMMAND_COUNT)
	{
		return -2;
	}

	packetData[0] = commandCount;

	int i;
	for (i = 0; i < commandCount; i++)
	{
		uint8_t* packetPointer = packetData
				+ MOTION_CONTROLLER_HID_REPORT_ID_SIZE
				+ i * MOTION_CONTROLLER_COMMAND_SIZE;

		*packetPointer = commands[i].command;
		*(packetPointer + 1) = commands[i].option;
		// rawData[4] 를 한번에 복사하기 위함
		*((uint32_t*) (packetPointer + 2)) = commands[i].data.uint32Data;
	}

	return packetSize;
}

