/*
 * HighPowerMotor.h
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#ifndef SRC_HIGHPOWERMOTOR_H_
#define SRC_HIGHPOWERMOTOR_H_

#include "Motor.h"

class HighPowerMotor: public Motor {
public:
	HighPowerMotor(){};
	HighPowerMotor(volatile uint32_t * pwm_channel_1,
			volatile uint32_t * pwm_channel_2,
			volatile uint32_t * encoder_cnt);

	virtual ~HighPowerMotor(){};

	void init(volatile uint32_t * pwm_channel_1,
			volatile uint32_t * pwm_channel_2,
			volatile uint32_t * encoder_cnt);

	virtual void setVelocity(const double& velocity);

private:
	volatile uint32_t * channelCW;
	volatile uint32_t * channelCCW;
};

#endif /* SRC_HIGHPOWERMOTOR_H_ */
