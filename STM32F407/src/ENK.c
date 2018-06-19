/*
 * ENK.c
 *
 */

#include "ENK.h"

//Quadratic signal
void ENK3init() {

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12,
			TIM_ICPolarity_Rising,
			TIM_ICPolarity_Rising);
	TIM_SetAutoreload(TIM1, 0xffff);

	TIM_SetCounter(TIM1, 0x7fff);

	TIM_Cmd(TIM1, ENABLE);
}

void ENK2init() {

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,
			TIM_ICPolarity_Rising,
			TIM_ICPolarity_Rising);
	TIM_SetAutoreload(TIM5, 0xffff);

	TIM_SetCounter(TIM5, 0x7fff);

	TIM_Cmd(TIM5, ENABLE);
}

//PWM signal
void ENK1init() {

	TIM_ICInitTypeDef TIM_ICInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICFilter = 10;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_PWMIConfig(TIM9, &TIM_ICInitStruct);

	TIM_SelectInputTrigger(TIM9, TIM_TS_TI1FP1 );

	TIM_SelectSlaveMode(TIM9, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM9, TIM_MasterSlaveMode_Enable);

	TIM_Cmd(TIM9, ENABLE);

	TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);
}

volatile uint16_t period;
volatile uint16_t pulse;
volatile uint16_t duty;

void TIM1_BRK_TIM9_IRQHandler(void){
	period = TIM_GetCapture1(TIM9);
	pulse= TIM_GetCapture2(TIM9);

	duty = (TIM_GetCapture2(TIM9) * 100) / TIM_GetCapture1(TIM9);
}
