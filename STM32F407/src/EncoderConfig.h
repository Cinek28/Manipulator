/*
 * EncoderConfig.h
 *
 */

#ifndef MANIPULATOR_ENK_H_
#define MANIPULATOR_ENK_H_

#include "stm32f4xx.h"

void ENK7init(void);
void ENK6init(void);
void ENK3init(void);
void ENK2init(void);
void ENK1init(void);

void TIM1_BRK_TIM9_IRQHandler();

#endif /* ENCODERCONFIG_H */
