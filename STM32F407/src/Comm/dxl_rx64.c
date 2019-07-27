/*
 * dxl_rx64.c
 *	File contains definitions of functions used to control
 *	dynamixel servos (the ones using protocol 1.0) over UART
 *	for STM32 microcontrollers.
 *
 *	Additional UART:RS-485 transceiver needed.
 *
 *	UART is set to half-duplex mode.
 *
 *	Library written using STDPeriph libraries for STM microcontrollers.
 *
 */

#include <stdint.h>

#include "dxl_rx64.h"

extern bool getAndCheckResponse (const uint8_t servoId,
						  	  	 ServoResponse* msg);

void sendServoByte(uint8_t byte)
{
	while(USART_GetFlagStatus(SERVO_USART_PORT, USART_FLAG_TXE) == RESET);
	USART_SendData(SERVO_USART_PORT, byte);
}

void sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t noOfParams,
                       const uint8_t *params)
{
	for(uint8_t i = 0; i < 3; ++i)
	{
		sendServoByte(0xff);
		sendServoByte(0xff);  // command header
		sendServoByte(servoId);  // servo ID
		uint8_t checksum = servoId;

		sendServoByte(noOfParams + 2);  // number of following bytes
		checksum += noOfParams + 2 + commandByte;

		sendServoByte((uint8_t)commandByte);
		for (uint8_t i = 0; i < noOfParams; i++)
		{
			sendServoByte (params[i]);  // parameters
			checksum += params[i];
		}
		__disable_irq();
		sendServoByte (~checksum);  // checksum
		__enable_irq();
	}
}

bool getServoResponseByte (cyclicBuffer* buffer,
						   ServoResponse* msg)
{
	setBufferByte(buffer, (uint8_t)USART_ReceiveData(SERVO_USART_PORT));
	uint8_t size = (uint8_t)getBufferOccupiedSize(buffer);
	if(size == 2)
	{
		if(peekBufferByte(buffer, 0) != 0xff ||
				peekBufferByte(buffer, 1) != 0xff)
		{
			clearBuffer(buffer);
		}
	}
	else if(size == 3)
	{
		msg->id = *(buffer->end);
	}
	else if(size == 4)
	{
		msg->length = *(buffer->end);
		if(msg->length > SERVO_MAX_PARAMS)
		{
			msg->length = 0;
			clearBuffer(buffer);
		}
	}
	else if(size == 5)
	{
		msg->error = *(buffer->end);
	}
	else if(size > 5 && size < 5 + msg->length)
	{
		msg->params[size-5] = *(buffer->end);
	}
	else
	{
		msg->checksum = *(buffer->end);
		uint8_t calcChecksum = msg->id + msg->error + msg->length;
		for(int i = 0; i < msg->length; ++i)
		{
			calcChecksum += msg->params[i];
			calcChecksum = ~calcChecksum;
		}

		clearBuffer(buffer);

		if(calcChecksum == msg->checksum)
		{
			return true;
		}
	}

	return false;
}

bool setServoDiode(uint8_t servoId,
				   uint8_t isOn)
{
	uint8_t params[2];
	params[0] =0x19;
	if(isOn){
		params[1] = 0x01;
		sendServoCommand(servoId, WRITE, 0x02, params);
	}else{
		params[1] = 0x00;
		sendServoCommand(servoId, WRITE, 0x02, params);
	}

	ServoResponse response;
	return getAndCheckResponse(servoId, &response);
}

// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque (const uint8_t servoId,
                     const uint16_t torqueValue)
{
    const uint8_t highByte = (uint8_t)((torqueValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(torqueValue & 0xff);

    if (torqueValue > 1023)
        return false;

    const uint8_t params[3] = {TORQUE,
                               lowByte,
                               highByte};

    sendServoCommand (servoId, WRITE, 3, params);

    ServoResponse response;
    return getAndCheckResponse (servoId, &response);
}

bool getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue)
{
    const uint8_t params[2] = {TORQUE,
                               2};  // read two bytes, starting at address TORQUE

    sendServoCommand (servoId, READ, 2, params);

    ServoResponse response;
    if (!getAndCheckResponse (servoId, &response))
        return false;

    *torqueValue = response.params[1];
    *torqueValue <<= 8;
    *torqueValue |= response.params[0];

    return true;
}

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool setServoMaxSpeed (const uint8_t servoId,
                       const uint16_t speedValue)
{
    const uint8_t highByte = (uint8_t)((speedValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(speedValue & 0xff);

    if (speedValue > 1023)
        return false;

    const uint8_t params[3] = {MAX_SPEED,
                               lowByte,
                               highByte};

    sendServoCommand (servoId, WRITE, 3, params);

    ServoResponse response;
    return getAndCheckResponse (servoId, &response);
}

bool getServoMaxSpeed (const uint8_t servoId,
                       uint16_t *speedValue)
{
    const uint8_t params[2] = {MAX_SPEED,
                               2};  // read two bytes, starting at address MAX_SPEED

    sendServoCommand (servoId, READ, 2, params);

    ServoResponse response;
    if (!getAndCheckResponse (servoId, &response))
        return false;

    *speedValue = response.params[1];
    *speedValue <<= 8;
    *speedValue |= response.params[0];

    return true;
}

bool getServoCurrentVelocity (const uint8_t servoId,
                              int16_t *velocityValue)
{
    const uint8_t params[2] = {CURRENT_SPEED,
                               2};  // read two bytes, starting at address CURRENT_SPEED

    sendServoCommand (servoId, READ, 2, params);

    ServoResponse response;
    if (!getAndCheckResponse (servoId, &response))
        return false;

    *velocityValue = response.params[1];
    *velocityValue <<= 8;
    *velocityValue |= response.params[0];

    return true;
}

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle (const uint8_t servoId,
                    const float angle)
{
    if (angle < 0 || angle > 300)
        return false;

    // angle values go from 0 to 0x3ff (1023)
    const uint16_t angleValue = (uint16_t)(angle * (1023.0 / 300.0));

    const uint8_t highByte = (uint8_t)((angleValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angleValue & 0xff);

    const uint8_t params[3] = {GOAL_ANGLE,
                               lowByte,
                               highByte};

    sendServoCommand (servoId, WRITE, 3, params);

    ServoResponse response;
    return getAndCheckResponse (servoId, &response);
}

bool getServoAngle (const uint8_t servoId,
                    uint16_t *angle)
{
    const uint8_t params[2] = {CURRENT_ANGLE,
                               2};  // read two bytes, starting at address CURRENT_ANGLE

    sendServoCommand (servoId, READ, 2, params);

    ServoResponse response;
    if (!getAndCheckResponse (servoId, &response))
        return false;

    uint16_t angleValue = response.params[1];
    angleValue <<= 8;
    angleValue |= response.params[0];

    *angle = angleValue;//(float)angleValue * 300.0 / 1023.0;

    return true;
}

bool enableTorque(uint8_t servoId,
				  uint8_t isEnabled)
{
	uint8_t params[2] = {0x18,isEnabled};
	sendServoCommand(servoId, WRITE, 2, params);

	ServoResponse response;
	return getAndCheckResponse (servoId, &response);
}

bool setServoBaudrate(uint8_t servoId,
					  uint8_t baudrate)
{
	uint8_t params[2] = {0x04, baudrate};
	sendServoCommand(servoId, WRITE, 2, params);

	ServoResponse response;
	return getAndCheckResponse (servoId, &response);
}

bool setServoID(uint8_t oldServoId,
				uint8_t newServoId)
{
	uint8_t params[2] = {0x03, newServoId};
	sendServoCommand(oldServoId, WRITE, 2, params);

	ServoResponse response;
	return getAndCheckResponse (oldServoId, &response);
}

bool setServoSpeed(uint8_t servoId,
				   uint8_t higherByte,
				   uint8_t lowerByte)
{
	uint8_t params[3] = {0x20,lowerByte, higherByte};
	sendServoCommand(servoId, WRITE, 0x03, params);

	ServoResponse response;
	return getAndCheckResponse (servoId, &response);
}
