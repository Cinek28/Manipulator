/*
 * BoardConfig.h
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#ifndef BOARDCONFIG_H
#define BOARDCONFIG_H

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

#define CORE_CLK_FREQ 144000000
#define TIMEOUT_MS 200
#define SYSTICK_FREQ 1000
#define SYSTIC_CNT CORE_CLK_FREQ/SYSTICK_FREQ
#define BAUDRATE 57600

#ifdef __cplusplus
extern "C" {
#endif

// Clock configuration and initialization:
uint8_t RCC_CONF();
void GPIO_MOTOR_CONF();
void USART2_CONF();
void USART1_DXL_CONF(uint32_t baudrate);
void USART1_CONF();
void TIM_PWM_CONF();

#ifdef __cplusplus
}
#endif

#endif /* BOARDCONFIG_H */
