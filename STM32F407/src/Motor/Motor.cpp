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

void Motor::setEncoderRange(const uint32_t& range)
{
	encoder.range = range;
}

double Motor::getVelocity()
{
	float velocity = 0;

	if (*encoder.currentCount <= 0.1*encoder.range && encoder.previousCount >= encoder.range*0.9
			&& encoder.previousCount <= encoder.range) {
		velocity = *encoder.currentCount - encoder.previousCount + encoder.range;
	} else if (*encoder.currentCount <= encoder.range && *encoder.currentCount >= encoder.range*0.9
			&& encoder.previousCount <= 0.1*encoder.range) {
		velocity = *encoder.currentCount - encoder.previousCount - encoder.range;
	} else {
		velocity = *encoder.currentCount - encoder.previousCount;
	}

	encoder.previousCount = *encoder.currentCount;
	return velocity / encoder.range * 1000 * 2 * 3.1415;
}

void Motor::calcPWM(const double& targetVelocity)
{

	if(encoder.currentCount == nullptr)
	{
		pwm = targetVelocity/(maxVel*ratio)*MAX_PWM_VALUE;
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


