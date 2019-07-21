/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 7.0.1   2018-02-18

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
#include "BoardConfig.h"
#include "ManipulatorStateMachine.h"


void send_char(char, USART_TypeDef*);
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
void send_string(const char*, USART_TypeDef*);
void sendStartStop(char isOn);


volatile RCC_ClocksTypeDef RCC_Clocks;
int main(void)
{
	ManipulatorStateMachine manipulator;
	manipulator.init();
	GPIO_SetBits(GPIOB, GPIO_Pin_2);
//  RCC_CONF();
//  GPIO_CONF();
//  USART_CONF();
//  NVIC_CONF();
//  //ENK3init();
//  //ENK2init();
//  TIM_CONF();
//  initCan();

//  uint8_t params[5] = {0x06, 0x00, 0x00, 0xFF, 0x03};
//  setServoDiode(0x01, 0x01);
//  enableTorque(0x01, 0x01);
//  setServoTorque(0x01, 1023);
//  sendServoCommand(0x01, WRITE, 0x05, params );
//  setServoAngle(0x01, angle1);
//
//  setServoDiode(0x02, 0x01);
//  enableTorque(0x02, 0x01);
//  setServoTorque(0x02, 1023);
//  sendServoCommand(0x02, WRITE, 0x05, params );
//  setServoAngle(0x02, angle2);

  while (1)
  {
  }
}

/* Private functions */

//Sending data over USART:

void send_char(char c, USART_TypeDef* USART)
{
    while (USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
    USART_SendData(USART, c);
}

void send_string(const char* s, USART_TypeDef* USART)
{
    while (*s)
        send_char(*s++, USART);
}

//void USART1_IRQHandler (void)
//{
//	// check if the USART1 receive interrupt flag was set
//	if (USART_GetITStatus (USART1, USART_IT_RXNE))
//	{
//		const uint8_t byte = (uint8_t)USART_ReceiveData (USART1); // grab the byte from the data register
//
//        receiveBufferEnd++;
//        if (receiveBufferEnd >= receiveBuffer + REC_BUFFER_LEN)
//            receiveBufferEnd = receiveBuffer;
//
//        *receiveBufferEnd = byte;
//	}
//}

void USART2_IRQHandler (void) {
	   if(USART_GetITStatus(USART2, USART_IT_RXNE))
	   {
		   keyTimeout = 0;
		   char received = USART_ReceiveData(USART2);
		   send_char(received, USART2);
		   switch(received)
		        {
		            case 'a':
		            	GPIO_SetBits(GPIOD, GPIO_Pin_0);
		            	GPIO_ResetBits(GPIOD, GPIO_Pin_1);
		            	TIM_SetCompare1(TIM2,400);
		                break;
		            case 'q':
		            	GPIO_ResetBits(GPIOD, GPIO_Pin_0);
		            	GPIO_SetBits(GPIOD, GPIO_Pin_1);
		            	TIM_SetCompare1(TIM2,400);
		                break;
		            case 'w':
		            	TIM_SetCompare3(TIM2,250);
		            	TIM_SetCompare4(TIM2,0);
		                break;
		            case 's':
		            	TIM_SetCompare3(TIM2,0);
		            	TIM_SetCompare4(TIM2,250);
		                break;
		            case 'e':
		            	GPIO_SetBits(GPIOD, GPIO_Pin_4);
		            	GPIO_ResetBits(GPIOD, GPIO_Pin_7);
		            	TIM_SetCompare2(TIM2,1000);
		                break;
		            case 'd':
		            	GPIO_ResetBits(GPIOD, GPIO_Pin_4);
		            	GPIO_SetBits(GPIOD, GPIO_Pin_7);
		            	TIM_SetCompare2(TIM2,1000);
		                break;
		            case 'r':
		            	GPIO_SetBits(GPIOE, GPIO_Pin_0);
		            	GPIO_ResetBits(GPIOE, GPIO_Pin_4);
		            	TIM_SetCompare4(TIM4,200);
		                break;
		            case 'f':
		            	GPIO_ResetBits(GPIOE, GPIO_Pin_0);
		            	GPIO_SetBits(GPIOE, GPIO_Pin_4);
		            	TIM_SetCompare4(TIM4,200);
		                break;
		            case 't':
		            	GPIO_ResetBits(GPIOE, GPIO_Pin_3);
		                GPIO_SetBits(GPIOE, GPIO_Pin_2);
		            	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		                GPIO_SetBits(GPIOC, GPIO_Pin_14);
		                TIM_SetCompare3(TIM4, 300);
		                break;
		            case 'g':
		            	GPIO_SetBits(GPIOE, GPIO_Pin_3);
		                GPIO_ResetBits(GPIOE, GPIO_Pin_2);
		            	GPIO_SetBits(GPIOC, GPIO_Pin_13);
		                GPIO_ResetBits(GPIOC, GPIO_Pin_14);
		            	TIM_SetCompare3(TIM4, 300);
		                break;
		            case 'h':
//		            	TIM_SetCompare1(TIM3,400);
//		            	TIM_SetCompare2(TIM3,400);
		            	break;
		            case 'u':
						angle1+= 1;
						if(angle1 > 300)
						{
							angle1 = 300;
						}
						setServoAngle(0x01, angle1);
						angle2-=1;
						if(angle2 < 0)
						{
							angle2 = 0;
						}
						setServoAngle(0x02,angle2);
		            	send_string("RS_SEND", USART2);
		            	break;
		            case 'j':
						angle1-= 1;
						if(angle1 < 0)
						{
							angle1 = 0;
						}
						setServoAngle(0x01, angle1);
						angle2+=1;
						if(angle2 > 300)
						{
							angle2 = 300;
						}
						setServoAngle(0x02,angle2);
		            	send_string("RS_SEND", USART2);
		            	break;
		            case 'i':
						angle1+= 1;
						if(angle1 > 300)
						{
							angle1 = 300;
						}
						setServoAngle(0x01, angle1);
						angle2+=1;
						if(angle2 > 300)
						{
							angle2 = 300;
						}
						setServoAngle(0x02,angle2);
		            	send_string("RS_SEND", USART2);
		            	break;
		            case 'k':
						angle1-= 1;
						if(angle1 < 0)
						{
							angle1 = 0;
						}
						setServoAngle(0x01, angle1);
						angle2-=1;
						if(angle2 < 0)
						{
							angle2 = 0;
						}
						setServoAngle(0x02,angle2);
		            	send_string("RS_SEND", USART2);
		            	break;

		            default:
		            	TIM_SetCompare1(TIM2,0);
		            	TIM_SetCompare2(TIM2,0);
		            	TIM_SetCompare3(TIM2,0);
		            	TIM_SetCompare4(TIM2,0);
		            	TIM_SetCompare1(TIM3,0);
		            	TIM_SetCompare2(TIM3,0);
		            	TIM_SetCompare3(TIM4,0);
		            	TIM_SetCompare4(TIM4,0);
		                break;
		        }
	      USART_ClearITPendingBit(USART2, USART_FLAG_RXNE);
	      GPIO_ResetBits(GPIOB, GPIO_Pin_2);
	   }
}

void CAN1_RX0_IRQHandler(void) {
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		//odebrane dane w strukturze 'rxMessage'
		CAN_Receive(CAN1, CAN_FIFO0, &rxMessage);
		int8_t temp = 0;
		switch (rxMessage.StdId) {
		case 101:
			//Motors start/stop:
			keyTimeout = 0;
			sendStartStop(rxMessage.Data[0]);
			break;
		case 160:
			//Set motors velocity:
			keyTimeout = 0;
			temp = rxMessage.Data[0];
			baseSpeed = temp*1000/100;
			if(baseSpeed < 0)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_0);
				GPIO_ResetBits(GPIOD, GPIO_Pin_1);
				baseSpeed = (-1)*baseSpeed;
			}
			else
			{
				GPIO_ResetBits(GPIOD, GPIO_Pin_0);
				GPIO_SetBits(GPIOD, GPIO_Pin_1);
			}
		    TIM_SetCompare1(TIM2,baseSpeed);

		    temp = rxMessage.Data[1];
		    int forearmSpeed = temp*1000/100;
			if(forearmSpeed < 0)
			{
				forearmSpeed = (-1)*forearmSpeed;
            	TIM_SetCompare3(TIM2,forearmSpeed);
            	TIM_SetCompare4(TIM2,0);
			}
			else
			{
            	TIM_SetCompare3(TIM2,0);
            	TIM_SetCompare4(TIM2,forearmSpeed);
			}

			temp = rxMessage.Data[2];
			armSpeed = temp*1000/100;
			if(armSpeed < 0)
			{
				armSpeed = (-1)*armSpeed;
            	GPIO_SetBits(GPIOD, GPIO_Pin_4);
            	GPIO_ResetBits(GPIOD, GPIO_Pin_7);
			}
			else
			{
            	GPIO_ResetBits(GPIOD, GPIO_Pin_4);
            	GPIO_SetBits(GPIOD, GPIO_Pin_7);
			}
			TIM_SetCompare2(TIM2,armSpeed);

			temp = rxMessage.Data[3];
			wristRotateSpeed = temp*1000/100;
			if(wristRotateSpeed < 0)
			{
				wristRotateSpeed = (-1)*wristRotateSpeed;
            	GPIO_SetBits(GPIOE, GPIO_Pin_0);
            	GPIO_ResetBits(GPIOE, GPIO_Pin_4);
			}
			else
			{
            	GPIO_ResetBits(GPIOE, GPIO_Pin_0);
            	GPIO_SetBits(GPIOE, GPIO_Pin_4);
			}
			TIM_SetCompare4(TIM4,wristRotateSpeed);

			temp = rxMessage.Data[4];
			wristPitchSpeed = (float)temp/10;
			if(wristPitchSpeed < 0)
			{
				wristPitchSpeed = (-1)*wristPitchSpeed;
				angle1+= wristPitchSpeed;
				if(angle1 > 300)
				{
					angle1 = 300;
				}
				setServoAngle(0x01, angle1);
				angle2-= wristPitchSpeed;
				if(angle2 < 0)
				{
					angle2 = 0;
				}
				setServoAngle(0x02,angle2);
//				uint16_t firstServoSpeed = ((1023*wristPitchSpeed)/100);
//				uint16_t secondServoSpeed = ((1023*wristPitchSpeed)/100) + 1024;
//            	setServoSpeed(0x02, (uint8_t)(firstServoSpeed >> 8), (uint8_t)(firstServoSpeed));
//            	setServoSpeed(0x01, (uint8_t)(secondServoSpeed >> 8), (uint8_t)(secondServoSpeed));
			}
			else if(wristPitchSpeed > 0)
			{
				angle1-= wristPitchSpeed;
				if(angle1 < 0)
				{
					angle1 = 0;
				}
				setServoAngle(0x01, angle1);
				angle2+=wristPitchSpeed;
				if(angle2 > 300)
				{
					angle2 = 300;
				}
				setServoAngle(0x02,angle2);
			}

			temp = rxMessage.Data[5];
			wristRollSpeed = (float)temp/10;
			if(wristRollSpeed < 0)
			{
				wristRollSpeed = (-1)*wristRollSpeed;
				angle1+= wristRollSpeed;
				if(angle1 > 300)
				{
					angle1 = 300;
				}
				setServoAngle(0x01, angle1);
				angle2+=wristRollSpeed;
				if(angle2 > 300)
				{
					angle2 = 300;
				}
				setServoAngle(0x02,angle2);
//				uint16_t firstServoSpeed = ((1023*wristRollSpeed)/100) + 1024;
//				uint16_t secondServoSpeed = ((1023*wristRollSpeed)/100) + 1024;
//            	setServoSpeed(0x02, (uint8_t)(firstServoSpeed >> 8), (uint8_t)(firstServoSpeed));
//            	setServoSpeed(0x01, (uint8_t)(secondServoSpeed >> 8), (uint8_t)(secondServoSpeed));
			}
			else if(wristRollSpeed > 0)
			{
				angle1-= wristRollSpeed;
				if(angle1 < 0)
				{
					angle1 = 0;
				}
				setServoAngle(0x01, angle1);
				angle2-=wristRollSpeed;
				if(angle2 < 0)
				{
					angle2 = 0;
				}
				setServoAngle(0x02,angle2);
//				uint16_t firstServoSpeed = ((1023*wristRollSpeed)/100);
//				uint16_t secondServoSpeed = ((1023*wristRollSpeed)/100);
//            	setServoSpeed(0x01, (uint8_t)(firstServoSpeed >> 8), (uint8_t)(firstServoSpeed));
//            	setServoSpeed(0x02, (uint8_t)(secondServoSpeed >> 8), (uint8_t)(secondServoSpeed));
			}

			temp = rxMessage.Data[6];
			gripperSpeed = temp*1000/300;
//			if(gripperSpeed < 0)
//			{
//				gripperSpeed = (-1)*gripperSpeed;
//            	GPIO_ResetBits(GPIOE, GPIO_Pin_3);
//                GPIO_SetBits(GPIOE, GPIO_Pin_2);
//            	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
//                GPIO_SetBits(GPIOC, GPIO_Pin_14);
//                TIM_SetCompare3(TIM4, gripperSpeed);
//            	TIM_SetCompare1(TIM3, gripperSpeed);
//			}
//			else
//			{
//            	GPIO_SetBits(GPIOE, GPIO_Pin_3);
//                GPIO_ResetBits(GPIOE, GPIO_Pin_2);
//            	GPIO_SetBits(GPIOC, GPIO_Pin_13);
//                GPIO_ResetBits(GPIOC, GPIO_Pin_14);
//                TIM_SetCompare1(TIM3, gripperSpeed);
//            	TIM_SetCompare3(TIM4, gripperSpeed);
//			}

			break;
		case 162:
			//Motors start/stop:
			keyTimeout = 0;
			sendStartStop(rxMessage.Data[0]);
			break;
		default:
			break;
		}
	}
}

void sendStartStop(char isOn)
{
	if(isOn != '1')
	{
    	TIM_SetCompare1(TIM2,0);
    	TIM_SetCompare2(TIM2,0);
    	TIM_SetCompare3(TIM2,0);
    	TIM_SetCompare4(TIM2,0);
//    	TIM_SetCompare1(TIM3,0);
//    	TIM_SetCompare2(TIM3,0);
    	TIM_SetCompare3(TIM4,0);
    	TIM_SetCompare4(TIM4,0);
	}
	else
	{

	}
}



void SysTick_Handler(void)
{
	if(keyTimeout++ > TIMEOUT)
	{
    	TIM_SetCompare1(TIM2,0);
    	TIM_SetCompare2(TIM2,0);
    	TIM_SetCompare3(TIM2,0);
    	TIM_SetCompare4(TIM2,0);
//    	TIM_SetCompare1(TIM3,0);
//    	TIM_SetCompare2(TIM3,0);
    	TIM_SetCompare3(TIM4,0);
    	TIM_SetCompare4(TIM4,0);
//    	uint8_t params[5] = {0x06, 0x00, 0xFF, 0x03};
//    	sendServoCommand(0x02, WRITE, 0x05, params );
//    	enableTorque(0x02, 0x00);
//    	sendServoCommand(0x01, WRITE, 0x05, params );
//    	enableTorque(0x01, 0x00);

		keyTimeout = 0;
	}
	TimingDelay_Decrement();
}

volatile uint16_t delayTimer;

void delay(uint16_t time){
	delayTimer= time;
	while(delayTimer!=0){
		;
	}
}

void TimingDelay_Decrement(void){
	if(delayTimer>0){
		delayTimer--;
	}
}
