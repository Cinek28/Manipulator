/*
 * ServoMotor.h
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#ifndef SRC_SERVOMOTOR_H_
#define SRC_SERVOMOTOR_H_

#include "Motor.h"
#include "dxl_rx64.h"

#define MAX_VELOCITY 11.93805207 //maximum servo velocity in radians


class ServoMotor: public Motor {
public:

	enum MODE
	{
		JOINT_MODE= 0,
		WHEEL_MODE
	};

	ServoMotor(){};
	ServoMotor(const uint8_t& servo_id):servoID(servo_id){};
	virtual ~ServoMotor(){};

	void init(const uint8_t& servo_id);

	virtual void setVelocity(const double& velocity);
	void setCurrentMode(const MODE& mode);
	void setLed(bool isOn);

	virtual double getVelocity();
	const uint8_t& getServoID() const{ return servoID;};

private:
	uint8_t servoID = 1;
	MODE currentMode = WHEEL_MODE;

};

#endif /* SRC_SERVOMOTOR_H_ */
