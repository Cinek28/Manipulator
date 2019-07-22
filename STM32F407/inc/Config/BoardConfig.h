/*
 * BoardConfig.h
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#ifndef BOARDCONFIG_H
#define BOARDCONFIG_H

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

//#include "CAN.h"

#define TIMEOUT 200 //in ms
#define SYSTICK_FREQ 100000000
#define TIMEOUT_CNT SYSTICK_FREQ*TIMEOUT/(1000)
#define BAUDRATE 1000000
/* Private variables */

#ifdef __cplusplus
extern "C" {
#endif
// Clock configuration and initialization:
void RCC_CONF();
void GPIO_MOTOR_CONF();
void USART2_CONF();
void USART1_DXL_CONF(uint32_t baudrate);
void USART1_CONF();
void TIM_PWM_CONF();

#ifdef __cplusplus
}
#endif

#endif /* BOARDCONFIG_H */