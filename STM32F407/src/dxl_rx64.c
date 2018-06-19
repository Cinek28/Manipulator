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
 *	Library written using HAL libraries for STM microcontrollers.
 *
 */

#include "dxl_rx64.h"

ServoResponse response;

uint8_t servoErrorCode = 0;

volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;

void sendServoByte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1,byte);
}

void sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t noOfParams,
                       const uint8_t *params)
{
	for(uint8_t i = 0; i < 25; ++i)
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
		sendServoByte (~checksum);  // checksum
	}
}

void clearServoReceiveBuffer (void)
{
    receiveBufferStart = receiveBufferEnd;
}

size_t getServoBytesAvailable (void)
{
    volatile uint8_t *start = receiveBufferStart;
    volatile uint8_t *end = receiveBufferEnd;

    if (end >= start)
        return (size_t)(end - start);
    else
        return (size_t)(REC_BUFFER_LEN - (start - end));
}

uint8_t getServoByte (void)
{
    receiveBufferStart++;
    if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
        receiveBufferStart = receiveBuffer;

    return *receiveBufferStart;
}


bool getServoResponse (void)
{
    uint8_t retries = 0;

    clearServoReceiveBuffer();

    while (getServoBytesAvailable() < 4)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries at start\n");
            #endif
            return false;
        }

        //mWaitus (REC_WAIT_START_US);
    }
    retries = 0;

    getServoByte();  // servo header (two 0xff bytes)
    getServoByte();

    response.id = getServoByte();
    response.length = getServoByte();

    if (response.length > SERVO_MAX_PARAMS)
    {
        #ifdef SERVO_DEBUG
        printf ("Response length too big: %d\n", (int)response.length);
        #endif
        return false;
    }

    while (getServoBytesAvailable() < response.length)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries waiting for params, got %d of %d params\n", getServoBytesAvailable(), response.length);
            #endif
            return false;
        }

        //mWaitus (REC_WAIT_PARAMS_US);
    }

    response.error = getServoByte();
    servoErrorCode = response.error;

    for (uint8_t i = 0; i < response.length - 2; i++)
        response.params[i] = getServoByte();


    uint8_t calcChecksum = response.id + response.length + response.error;
    for (uint8_t i = 0; i < response.length - 2; i++)
        calcChecksum += response.params[i];
    calcChecksum = ~calcChecksum;

    const uint8_t recChecksum = getServoByte();
    if (calcChecksum != recChecksum)
    {
        #ifdef SERVO_DEBUG
        printf ("Checksum mismatch: %x calculated, %x received\n", calcChecksum, recChecksum);
        #endif
        return false;
    }

    return true;
}


inline bool getAndCheckResponse (const uint8_t servoId)
{
    if (!getServoResponse())
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Servo %d did not respond correctly or at all\n", (int)servoId);
        #endif
        return false;
    }

    if (response.id != servoId)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response ID %d does not match command ID %d\n", (int)response.id);
        #endif
        return false;
    }

    if (response.error != 0)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response error code was nonzero (%d)\n", (int)response.error);
        #endif
        return false;
    }

    return true;
}

void setServoDiode(uint8_t servoID, uint8_t isOn){
	uint8_t params[2];
	params[0] =0x19;
	if(isOn){
		params[1] = 0x01;
		sendServoCommand(servoID, WRITE, 0x02, params);
	}else{
		params[1] = 0x00;
		sendServoCommand(servoID, WRITE, 0x02, params);
	}
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

//    if (!getAndCheckResponse (servoId))
//        return false;
//
//    return true;
}

bool getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue)
{
    const uint8_t params[2] = {TORQUE,
                               2};  // read two bytes, starting at address TORQUE

    sendServoCommand (servoId, READ, 2, params);

    if (!getAndCheckResponse (servoId))
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

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}

bool getServoMaxSpeed (const uint8_t servoId,
                       uint16_t *speedValue)
{
    const uint8_t params[2] = {MAX_SPEED,
                               2};  // read two bytes, starting at address MAX_SPEED

    sendServoCommand (servoId, READ, 2, params);

    if (!getAndCheckResponse (servoId))
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

    if (!getAndCheckResponse (servoId))
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

//    if (!getAndCheckResponse (servoId))
//        return false;
//
//    return true;
}

bool getServoAngle (const uint8_t servoId,
                    uint16_t *angle)
{
    const uint8_t params[2] = {CURRENT_ANGLE,
                               2};  // read two bytes, starting at address CURRENT_ANGLE

    sendServoCommand (servoId, READ, 2, params);

    if (!getAndCheckResponse (servoId))
        return false;

    uint16_t angleValue = response.params[1];
    angleValue <<= 8;
    angleValue |= response.params[0];

    *angle = angleValue;//(float)angleValue * 300.0 / 1023.0;

    return true;
}

void enableTorque(uint8_t servoID, uint8_t isEnabled)
{
	uint8_t params[2] = {0x18,isEnabled};
	sendServoCommand(servoID, WRITE, 2, params);
}

void setServoBaudrate(uint8_t servoID, uint8_t baudrate)
{
	uint8_t params[2] = {0x04, baudrate};
	sendServoCommand(servoID, WRITE, 2, params);
}

void setServoID(uint8_t oldServoID, uint8_t newServoID)
{
	uint8_t params[2] = {0x03, newServoID};
	sendServoCommand(oldServoID, WRITE, 2, params);
}

void setServoSpeed(uint8_t servoID, uint8_t higherByte, uint8_t lowerByte){
	uint8_t params[3] = {0x20,lowerByte, higherByte};
	sendServoCommand(servoID, WRITE, 0x03, params);
}


