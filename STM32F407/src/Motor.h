/*
 * Motor.h
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include <stdint.h>

#define MAX_PWM_VALUE 1000

class Motor
{
public:

	virtual ~Motor(){};

	virtual void initPID(const uint16_t& k_p, uint16_t k_d,
			const uint16_t& k_i);

	virtual void setRatio(const double& ratio);
	virtual void setMaxVelocity(const double& max_vel);
    virtual void setVelocity(const double& velocity);

    virtual const double getVelocity() const;

protected:
    double ratio = 1;

    volatile uint32_t * encoderCnt;

    int16_t prevError = 0;
    int16_t errorSum = 0;
    uint16_t Kp, Kd, Ki;

    int16_t pwm = 0;
    double maxVel;

    virtual void calcPWM(const double& targetVelocity);
};

#endif /* SRC_MOTOR_H_ */
