/*
 * PWMMotor.cpp
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#include "PWMMotor.h"

PWMMotor::PWMMotor(volatile uint32_t * pwm_channel,
		volatile uint32_t * encoder_cnt,
		GPIO_TypeDef * port, uint16_t pin_1,
		uint16_t pin_2)
{
	pwmChannel = pwm_channel;
	encoderCnt = encoder_cnt;
	gpioPort = port;
	pinCW = pin_1;
	pinCCW = pin_2;
}

void PWMMotor::init(volatile uint32_t * pwm_channel,
		volatile uint32_t * encoder_cnt,
		GPIO_TypeDef * port, uint16_t pin_1,
		uint16_t pin_2)
{
	pwmChannel = pwm_channel;
	encoderCnt = encoder_cnt;
	gpioPort = port;
	pinCW = pin_1;
	pinCCW = pin_2;
}

void PWMMotor::setVelocity(const double& velocity)
{
	calcPWM(velocity*ratio);

	if(pwm < 0)
	{
		pwm = -1*pwm;
		GPIO_ResetBits(gpioPort, pinCW);
		GPIO_SetBits(gpioPort, pinCCW);
	}
	else
	{
		GPIO_SetBits(gpioPort, pinCW);
		GPIO_ResetBits(gpioPort, pinCCW);
	}

	*pwmChannel = pwm;
}
