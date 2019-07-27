/*
 * UARTComm.c
 *
 *  Created on: Jul 25, 2019
 *      Author: mrbr
 */

#include "UARTComm.h"
#include "task.h"

bool getAndCheckResponse (const uint8_t servoId, ServoResponse* msg)
{
	return xQueueReceive(rs485Queue, (void*) msg, 100/portTICK_PERIOD_MS);
}

void USART1_IRQHandler (void)
{
	if (USART_GetITStatus (USART1, USART_IT_RXNE))
	{
		BaseType_t taskWoken = pdFALSE;
		if(getServoResponseByte(&servoBuffer, &servoMsg))
			xQueueSendFromISR(rs485Queue, &msg, &taskWoken);

        USART_ClearITPendingBit(USART1, USART_FLAG_RXNE);
        portYIELD_FROM_ISR(taskWoken);
	}
}

void USART2_IRQHandler (void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE))
	{
	    setBufferByte(&msgBuffer, (uint8_t)USART_ReceiveData(USART2));
	    uint8_t size = (uint8_t)getBufferOccupiedSize(&msgBuffer);
	    BaseType_t taskWoken = pdFALSE;
	    if(size == 2)
	    {
	    	if(peekBufferByte(&msgBuffer, 0) != 0xff ||
	    	   peekBufferByte(&msgBuffer, 1) != 0xff)
	    	{
	    		clearBuffer(&msgBuffer);
	    	}
	    }
	    else if(size == 3)
	    {
	    	msg.type = *(msgBuffer.end);
	    }
	    else if(size == 4)
	    {
	    	msg.length = *(msgBuffer.end);
	    	if(msg.length > MANIPULATOR_MAX_PARAMS)
	    	{
	    		msg.length = 0;
	    		clearBuffer(&msgBuffer);
	    	}
	    }
	    else if(size > 4 && size < 4 + msg.length)
	    {
	    	msg.params[size-5] = *(msgBuffer.end);
	    }
	    else
	    {
	    	msg.checksum = *(msgBuffer.end);
	    	uint8_t calcChecksum = msg.type + msg.length;
	    	for(int i = 0; i < msg.length; ++i)
	    	{
	    		calcChecksum += msg.params[i];
	    		calcChecksum = ~calcChecksum;
	    	}
	    	if(!(calcChecksum != msg.checksum ||
	    			(msg.type == CHANGE_STATE && msg.length != 1)))
	    		xQueueOverwriteFromISR(usartQueue, &msg, &taskWoken);

	    	clearBuffer(&msgBuffer);
	    }

	    USART_ClearITPendingBit(USART2, USART_FLAG_RXNE);
	    portYIELD_FROM_ISR(taskWoken);
	}
}
