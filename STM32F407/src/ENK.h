/*
 * ENK.h
 *
 */

#include "stm32f4xx.h"

#ifndef MANIPULATOR_ENK_H_
#define MANIPULATOR_ENK_H_

void ENK3init(void);
void ENK2init(void);
void ENK1init(void);

void TIM1_BRK_TIM9_IRQHandler();

#endif /* MANIPULATOR_ENK_H_ */
