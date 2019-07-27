/*
 * ServoMotor.cpp
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#include "ServoMotor.h"

void ServoMotor::init(const uint8_t& servo_id)
{
	servoID = servo_id;
	setServoDiode(servoID, 0x01);
	setServoTorque(servoID, 1023);
	enableTorque(servoID, 0x01);
}

void ServoMotor::setVelocity(const double& velocity)
{
	double vel = velocity * ratio;
	if(vel > MAX_VELOCITY)
		vel = MAX_VELOCITY;
	else if(vel < -MAX_VELOCITY)
		vel = - MAX_VELOCITY;

	int16_t vel_cmd = (vel/MAX_VELOCITY*1023);

	if(vel_cmd > 0)
		vel_cmd += 1023;
	else
		vel_cmd = -1*vel;

	setServoSpeed(servoID, (uint8_t)(vel_cmd >> 8), (uint8_t)(vel_cmd));
}

double ServoMotor::getVelocity()
{
	int16_t value;
	getServoCurrentVelocity(servoID, &value);
	return (double)value; //TODO
}

void ServoMotor::setCurrentMode(const ServoMotor::MODE& mode)
{

}

void ServoMotor::setLed(bool isOn)
{
	if(isOn)
		setServoDiode(servoID, 0x01);
	else
		setServoDiode(servoID, 0x00);
}


