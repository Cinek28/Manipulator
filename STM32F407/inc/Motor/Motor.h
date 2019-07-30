/*
 * PWMMotor.h
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include <stdint.h>
#include "EncoderConfig.h"

#define MAX_PWM_VALUE 1000

typedef enum MODE
{
	FREE = 0,
	PID
};

class Motor
{
public:

	virtual ~Motor(){};

	virtual void initPID(const uint16_t& k_p,
						 uint16_t k_d,
						 const uint16_t& k_i);

	virtual void setRatio(const double& ratio);
	virtual void setMaxVelocity(const double& max_vel);
	virtual void setEncoderRange(const uint32_t& range);
    virtual void setVelocity(const double& velocity) = 0;
    virtual void setMode(MODE newMode){mode = newMode;};

    virtual double getVelocity();

protected:
    double ratio = 1;
    MODE mode = FREE;

    Encoder encoder;

    int16_t prevError = 0;
    int16_t errorSum = 0;
    uint16_t Kp, Kd, Ki;

    int16_t pwm = 0;
    double maxVel;

    double currentVel;

    virtual void calcPWM(const double& targetVelocity,
    					 const double& currentVelocity);
};

#endif /* SRC_MOTOR_H_ */
