/*
******************************************************************************
File:     main.cpp
Info:     Generated by Atollic TrueSTUDIO(R) 7.0.1   2019-07-27

The MIT License (MIT)
Copyright (c) 2009-2016 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BoardConfig.h"
#include "UARTComm.h"
#include "Utility.h"
#include "ManipulatorStateMachine.h"
#include "EncoderConfig.h"

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

TaskHandle_t* stateMachineHandle = NULL;
TaskHandle_t* wakeTaskHandle = NULL;
void stateMachineHandler(void* a);

QueueHandle_t usartQueue;
QueueHandle_t rs485Queue;

cyclicBuffer msgBuffer;
volatile ManipulatorMsg msg;

cyclicBuffer servoBuffer;
volatile ServoResponse servoMsg;

int main(void)
{
  RCC_CONF();
  SystemCoreClockUpdate();

  clearBuffer(&msgBuffer);
  clearBuffer(&servoBuffer);

  usartQueue = xQueueCreate(1, sizeof(ManipulatorMsg*));
  rs485Queue = xQueueCreate(1, sizeof(ServoResponse*));

  if(usartQueue != NULL && rs485Queue != NULL)
  {
	  xTaskCreate(stateMachineHandler, "StateMachine",
			  	  20*configMINIMAL_STACK_SIZE,
				  NULL, 2, stateMachineHandle);
	  wakeTaskHandle = stateMachineHandle;

	  vTaskStartScheduler();
  }

  /* Infinite loop */
  while (1)
  {
  }
}

void stateMachineHandler(void*)
{
	ManipulatorStateMachine manipulator;
	manipulator.init();
	while(manipulator.run());
	manipulator.emergencyStop();
	_ERR("State machine broke. Fatal error. Reset device.")
	vTaskDelete(NULL);
}

#ifdef __cplusplus
extern "C" {
#endif

void vApplicationIdleHook( void )
{
	__WFI();
}

void vApplicationTickHook( void )
{

}

#ifdef __cplusplus
}
#endif
