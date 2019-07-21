/*
 * Motor.cpp
 *
 *  Created on: Jul 17, 2019
 *      Author: mrbr
 */

#include "Motor.h"

void Motor::initPID(const uint16_t& k_p, uint16_t k_d, const uint16_t& k_i)
{
	Kp = k_p;
	Kd = k_d;
	Ki = k_i;
}

void Motor::setRatio(const double& ratio)
{
	this->ratio = ratio;
}

void Motor::setMaxVelocity(const double& max_vel)
{
	maxVel = max_vel;
}

const double Motor::getVelocity() const
{
	return 0;
}

void Motor::calcPWM(const double& targetVelocity)
{

	if(encoderCnt == nullptr)
	{
		pwm = targetVelocity/maxVel*MAX_PWM_VALUE; //TODO
		return;
	}

	int16_t error = targetVelocity - getVelocity();
	errorSum += error;
	if (errorSum > MAX_PWM_VALUE)
		errorSum = MAX_PWM_VALUE;
	else if (errorSum < -MAX_PWM_VALUE)
		errorSum = -MAX_PWM_VALUE;

	pwm += Kp*error+Kd*(error-prevError)+Ki*errorSum;
	if (pwm > MAX_PWM_VALUE)
		pwm = MAX_PWM_VALUE;
	else if (pwm < -MAX_PWM_VALUE)
		pwm = -MAX_PWM_VALUE;
	prevError = error;
}

void Motor::setVelocity(const double& velocity)
{
	pwm = velocity;
}

