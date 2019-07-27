/*
 * UARTComm.h
 *
 *  Created on: Jul 25, 2019
 *      Author: mrbr
 */

#ifndef UARTCOMM_H_
#define UARTCOMM_H_

#include <stdlib.h>
#include <stdbool.h>

#include "stm32f4xx_usart.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "CyclicBuffer.h"
#include "dxl_rx64.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MANIPULATOR_MAX_PARAMS (BUF_MAX_LEN-3)

typedef enum ManipulatorCmd
{
	START = 1,
	MOVE = 2,
	CHANGE_STATE = 3,
	STOP = 4,
	IDLE = 5
} ManipulatorCmd;

typedef struct ManipulatorMsg
{
	ManipulatorCmd type;
    uint8_t length;
    uint8_t params[MANIPULATOR_MAX_PARAMS];
    uint8_t checksum;
} ManipulatorMsg;

typedef struct DXLMsg
{
	ManipulatorCmd type;
    uint8_t length;
    uint8_t params[MANIPULATOR_MAX_PARAMS];
    uint8_t checksum;
} DXLMsg;

extern QueueHandle_t usartQueue;
extern QueueHandle_t rs485Queue;

extern cyclicBuffer msgBuffer;
extern volatile ManipulatorMsg msg;

extern cyclicBuffer servoBuffer;
extern volatile ServoResponse servoMsg;

bool getAndCheckResponse (const uint8_t servoId, ServoResponse* msg);
void USART1_IRQHandler (void);
void USART2_IRQHandler (void);

#ifdef __cplusplus
}
#endif


#endif /* UARTCOMM_H_ */
