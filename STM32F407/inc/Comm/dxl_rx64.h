/*
 * dxl_rx64.h
 * File contains declarations of structures and functions
 * for dynamixel api for STM32 microcontrollers using
 * Protocol 1.0 for communication.
 *
 * Additional UART:RS-485 transceiver needed.
 *
 * UART is set to half-duplex mode.
 *
 * Library written using STDPeriph libraries for STM microcontrollers.
 *
 */

#ifndef DXL_RX64_H_
#define DXL_RX64_H_

#include <stdbool.h>
#include <stdlib.h>

#include "stm32f4xx_usart.h"

#include "CyclicBuffer.h"

#define REC_BUFFER_LEN 32
#define SERVO_MAX_PARAMS (REC_BUFFER_LEN - 5)

#define REC_WAIT_START_US    75
#define REC_WAIT_PARAMS_US   (SERVO_MAX_PARAMS * 5)
#define REC_WAIT_MAX_RETRIES 200

#define SERVO_INSTRUCTION_ERROR   (1 << 6)
#define SERVO_OVERLOAD_ERROR      (1 << 5)
#define SERVO_CHECKSUM_ERROR      (1 << 4)
#define SERVO_RANGE_ERROR         (1 << 3)
#define SERVO_OVERHEAT_ERROR      (1 << 2)
#define SERVO_ANGLE_LIMIT_ERROR   (1 << 1)
#define SERVO_INPUT_VOLTAGE_ERROR (1)

#define RETURN_DELAY        0x05
#define BLINK_CONDITIONS    0x11
#define SHUTDOWN_CONDITIONS 0x12
#define TORQUE              0x22
#define MAX_SPEED           0x20
#define CURRENT_SPEED       0x26
#define GOAL_ANGLE          0x1e
#define CURRENT_ANGLE 		0x24

#define SERVO_USART_PORT USART1

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ServoResponse
{
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t checksum;
} ServoResponse;


typedef enum ServoCommand
{
	PING = 1,
	READ = 2,
	WRITE = 3
} ServoCommand;

void sendServoByte(const uint8_t byte);

void sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params);

bool getServoResponseByte (cyclicBuffer* buffer,
						   ServoResponse* msg);

bool pingServo(const uint8_t servoId);

// Instructions sent to servo:
// Light up/down diode:
bool setServoDiode(uint8_t servoId,
				   uint8_t isOn);

bool setServoBaudrate(uint8_t servoId,
					  uint8_t baudrate);

// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque (const uint8_t servoId,
                     const uint16_t torqueValue);

bool getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue);

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool setServoMaxSpeed (const uint8_t servoId,
                       const uint16_t speedValue);

bool getServoMaxSpeed (const uint8_t servoId,
                       uint16_t *speedValue);

bool getServoCurrentVelocity (const uint8_t servoId,
                              int16_t *velocityValue);

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle (const uint8_t servoId,
                    const float angle);

bool getServoAngle (const uint8_t servoId,
					uint16_t *angle);

bool enableTorque(uint8_t servoId,
		          uint8_t isEnabled);

bool setServoSpeed(uint8_t servoId,
				   uint8_t higherByte,
				   uint8_t lowerByte);

#ifdef __cplusplus
}
#endif

#endif /* DXL_RX64_H_ */
