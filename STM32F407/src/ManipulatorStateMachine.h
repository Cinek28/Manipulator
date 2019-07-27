/*
 * ManipulatorStateMachine.h
 *
 *  Created on: Jul 11, 2019
 *      Author: mrbr
 */

#ifndef SRC_MANIPULATORSTATEMACHINE_H_
#define SRC_MANIPULATORSTATEMACHINE_H_

#include "BoardConfig.h"

#include "HighPowerMotor.h"
#include "ServoMotor.h"
#include "PWMMotor.h"

#include "UARTComm.h"

#define NUM_JOINTS 6
#define NUM_STATES 6
#define TIMEOUT 100

enum State
{
	INIT_STATE = 0,
	RESET_STATE,
	IDLE_STATE,
	JOINT_STATE,
	TOOL_STATE,
	SIM_STATE
};

typedef State (*handler_t)(void*);

class ManipulatorStateMachine
{
public:

	ManipulatorStateMachine();
	virtual ~ManipulatorStateMachine(){};

	uint8_t init();

	void setJointsVel(const double* velocities);
	void emergencyStop();

	void closeGripper(const double& velocity);
	void openGripper(const double& velocity);

	bool runOnce();
	bool run();


private:
	Motor* joints[NUM_JOINTS];

	PWMMotor shoulder;
	HighPowerMotor forearm;
	PWMMotor arm;
	PWMMotor wristRotation;
	ServoMotor wristPitch;
	ServoMotor gripperRotation;
	PWMMotor gripper;

	State currentState = INIT_STATE;
	State endState = RESET_STATE;
	handler_t stateHandlers[NUM_STATES];

	State getCurrentState(void);
};

State runInit(void* params);
State runReset(void* params);
State runIdle(void* params);
State runJointMode(void* params);
State runToolMode(void* params);
State runSimMode(void* params);

#endif /* SRC_MANIPULATORSTATEMACHINE_H_ */
