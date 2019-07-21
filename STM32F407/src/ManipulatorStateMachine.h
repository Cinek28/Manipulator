/*
 * ManipulatorStateMachine.h
 *
 *  Created on: Jul 11, 2019
 *      Author: mrbr
 */

#ifndef SRC_MANIPULATORSTATEMACHINE_H_
#define SRC_MANIPULATORSTATEMACHINE_H_

#include <vector>
#include <memory>

#include "Utils/StateMachine.h"

#include "Motor/PWMMotor.h"
#include "Motor/HighPowerMotor.h"
#include "Motor/ServoMotor.h"

#include "BoardConfig.h"

#define NUM_JOINTS 6

class ManipulatorStateMachine: public StateMachine
{
public:
	ManipulatorStateMachine();
	virtual ~ManipulatorStateMachine(){};

	const uint8_t init();

	void setJointsVel(const std::vector<double>& velocities);

	void closeGripper(const double& velocity);
	void openGripper(const double& velocity)

private:
	std::vector<std::unique_ptr<Motor>> joints[NUM_JOINTS];
	PWMMotor gripper;
};

#endif /* SRC_MANIPULATORSTATEMACHINE_H_ */
