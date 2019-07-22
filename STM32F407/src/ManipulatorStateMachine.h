/*
 * ManipulatorStateMachine.h
 *
 *  Created on: Jul 11, 2019
 *      Author: mrbr
 */

#ifndef SRC_MANIPULATORSTATEMACHINE_H_
#define SRC_MANIPULATORSTATEMACHINE_H_

//#include "Utils/StateMachine.h"

#include "BoardConfig.h"
#include "HighPowerMotor.h"
#include "ServoMotor.h"
#include "PWMMotor.h"

#define NUM_JOINTS 6

class ManipulatorStateMachine
{
public:
	ManipulatorStateMachine();
	virtual ~ManipulatorStateMachine(){};

	const uint8_t init();

	void setJointsVel(const double* velocities);

	void closeGripper(const double& velocity);
	void openGripper(const double& velocity);

private:
	Motor* joints[NUM_JOINTS];
	PWMMotor shoulder;
	HighPowerMotor forearm;
	PWMMotor arm;
	PWMMotor wristRotation;
	ServoMotor wristPitch;
	ServoMotor gripperRotation;
	PWMMotor gripper;
};

#endif /* SRC_MANIPULATORSTATEMACHINE_H_ */
