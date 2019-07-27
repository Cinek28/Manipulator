/*
 * ManipulatorStateMachine.cpp
 *
 *  Created on: Jul 11, 2019
 *      Author: mrbr
 */

#include <stdio.h>

#include "ManipulatorStateMachine.h"
#include "Debug.h"

ManipulatorStateMachine::ManipulatorStateMachine()
{
	stateHandlers[INIT_STATE] = runInit;
	stateHandlers[RESET_STATE] = runReset;
	stateHandlers[IDLE_STATE] = runIdle;
	stateHandlers[JOINT_STATE] = runJointMode;
	stateHandlers[TOOL_STATE] = runToolMode;
	stateHandlers[SIM_STATE] = runSimMode;

	joints[0] = &shoulder;
	joints[1] = &forearm;
	joints[2] = &arm;
	joints[3] = &wristRotation;
	joints[4] = &wristPitch;
	joints[5] = &gripperRotation;

	shoulder.init(&TIM2->CCR1, &TIM5->CNT, GPIOD, GPIO_Pin_0, GPIO_Pin_1);
	shoulder.setMaxVelocity(8.378);
	shoulder.setEncoderRange(ENCODER_RANGE_HIGH);
	shoulder.setRatio(0.5);

	forearm.init(&TIM2->CCR3, &TIM2->CCR4, &TIM3->CNT);
	forearm.setMaxVelocity(8.378);
	forearm.setEncoderRange(ENCODER_RANGE_LOW);
	forearm.setRatio(1.0);

	arm.init(&TIM2->CCR2, &TIM1->CNT, GPIOD, GPIO_Pin_4, GPIO_Pin_7);
	arm.setMaxVelocity(1.0);
	arm.setEncoderRange(ENCODER_RANGE_LOW);
	arm.setRatio(1.0);

	wristRotation.init(&TIM4->CCR4, &TIM8->CNT, GPIOE, GPIO_Pin_0, GPIO_Pin_4);
	wristRotation.setMaxVelocity(8.378);
	wristRotation.setEncoderRange(ENCODER_RANGE_HIGH);
	wristRotation.setRatio(1.0);

	wristPitch.init(1);
	wristPitch.setMaxVelocity(6.702);
	wristPitch.setRatio(1.0);

	gripperRotation.init(2);
	gripperRotation.setMaxVelocity(6.702);
	gripperRotation.setRatio(1.0);

	gripper.init(&TIM4->CCR3, nullptr, GPIOE, GPIO_Pin_2, GPIO_Pin_3);
	gripper.setMaxVelocity(8.378);
	gripper.setRatio(1.0);
}

uint8_t ManipulatorStateMachine::init()
{
	GPIO_MOTOR_CONF();
	TIM_PWM_CONF();
	USART1_CONF();
	USART1_DXL_CONF(BAUDRATE);
	USART2_CONF();
	_INFO("Initialized main board peripherals.");
	_INFO("Initialized manipulator motors.");
	return 0;
}

void ManipulatorStateMachine::setJointsVel(const double* velocities)
{
	for(int i = 0; i < NUM_JOINTS; ++i)
	{
		joints[i]->setVelocity(velocities[i]);
	}
}

void ManipulatorStateMachine::emergencyStop()
{
	for(int i = 0; i < NUM_JOINTS; ++i)
	{
		joints[i]->setVelocity(0.0);
	}
	_WARN("Initialized manipulator motors.");
}

void ManipulatorStateMachine::closeGripper(const double& velocity)
{
	gripper.setVelocity(velocity);
}

void ManipulatorStateMachine::openGripper(const double& velocity)
{
	gripper.setVelocity(-velocity);
}

State ManipulatorStateMachine::getCurrentState()
{
	return currentState;
}

bool ManipulatorStateMachine::run()
{
	if(currentState != INIT_STATE)
	{
		_ERR("Manipulator not initialized!");
	}
		return false;

	_INFO("Starting manipulator control.");
	while(!runOnce());
	return true;
}

bool ManipulatorStateMachine::runOnce()
{
	State tempState = stateHandlers[currentState]((void*)this);

	if(currentState == endState)
	{
		_WARN("End state met- exiting state machine");
		return true;
	}

	if(tempState != currentState)
	{
		_WARN("Changing state from %d to %d", currentState, tempState);
		currentState = tempState;
	}
	return false;
}

State runInit(void* params)
{
	ManipulatorStateMachine* manipulator = (ManipulatorStateMachine*)params;
	if(!manipulator->init())
		return RESET_STATE;

	return JOINT_STATE;
}

State runReset(void* params)
{
	ManipulatorStateMachine* manipulator = (ManipulatorStateMachine*)params;
	// TODO
	return RESET_STATE;
}

State runIdle(void* params)
{
	ManipulatorStateMachine* manipulator = (ManipulatorStateMachine*)params;
	manipulator->emergencyStop();
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	ManipulatorMsg msg;
	if(xQueueReceive(usartQueue, (void*) &msg, TIMEOUT/portTICK_PERIOD_MS))
	{
		switch(msg.type)
		{
			case CHANGE_STATE:
			{
				return (State)msg.params[0];
			}
			default:
				return IDLE_STATE;
		}
	}
	return IDLE_STATE;
}

State runJointMode(void* params)
{
	ManipulatorStateMachine* manipulator = (ManipulatorStateMachine*)params;
	ManipulatorMsg msg;
	if(xQueueReceive(usartQueue, (void*) &msg, TIMEOUT/portTICK_PERIOD_MS))
	{
		switch(msg.type)
		{
			case MOVE:
			{
				double vels[NUM_JOINTS];
				for(int i = 0; i < msg.length/2; ++i)
				{
					vels[i] = (double)(msg.params[i] | msg.params[i+1] << 8);
					manipulator->setJointsVel(vels);
				}
				return JOINT_STATE;
			}
			case CHANGE_STATE:
			{
				manipulator->emergencyStop();
				return (State)msg.params[0];
			}
			default:
				return IDLE_STATE;
		}
	}
	return IDLE_STATE;
}

State runToolMode(void* params)
{
	ManipulatorStateMachine* manipulator = (ManipulatorStateMachine*)params;
	return TOOL_STATE;
}

State runSimMode(void* params)
{
	ManipulatorStateMachine* manipulator = (ManipulatorStateMachine*)params;
	return SIM_STATE;
}

