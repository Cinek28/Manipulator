/*
 * EncoderConfig.h
 *
 */

#ifndef MANIPULATOR_ENK_H_
#define MANIPULATOR_ENK_H_

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_RANGE_LOW 1024
#define ENCODER_RANGE_HIGH 8400

typedef struct Encoder
{
	volatile uint32_t * currentCount;
	uint32_t previousCount;
	uint32_t range;
}Encoder;

void ENC7init(void);
void ENC6init(void);
void ENC3init(void);
void ENC2init(void);
void ENC1init(void);

void TIM1_BRK_TIM9_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif /* ENCODERCONFIG_H */
