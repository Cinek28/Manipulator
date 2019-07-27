/*
 * HighPowerMotor.cpp
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#include "HighPowerMotor.h"

HighPowerMotor::HighPowerMotor(volatile uint32_t * pwm_channel_1,
		volatile uint32_t * pwm_channel_2,
		volatile uint32_t * encoder_cnt)
{
	encoder.currentCount= encoder_cnt;
	channelCW = pwm_channel_1;
	channelCCW = pwm_channel_2;
}

void HighPowerMotor::init(volatile uint32_t * pwm_channel_1,
		volatile uint32_t * pwm_channel_2,
		volatile uint32_t * encoder_cnt)
{
	encoder.currentCount= encoder_cnt;
	channelCW = pwm_channel_1;
	channelCCW = pwm_channel_2;
}

void HighPowerMotor::setVelocity(const double& velocity)
{
	calcPWM(velocity*ratio);

	if(pwm < 0)
	{
		pwm = -1*pwm;
		*channelCW = 0;
		*channelCCW = pwm;
	}
	else
	{
		*channelCW = pwm;
		*channelCCW = 0;
	}
}
