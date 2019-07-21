/*
 * ManipulatorStateMachine.cpp
 *
 *  Created on: Jul 11, 2019
 *      Author: mrbr
 */

#include "ManipulatorStateMachine.h"

ManipulatorStateMachine::ManipulatorStateMachine()
{
	joints[0] = &shoulder;
	joints[1] = &forearm;
	joints[2] = &arm;
	joints[3] = &wristRotation;
	joints[4] = &wristPitch;
	joints[5] = &gripperRotation;

	shoulder.init(&TIM2->CCR1, &TIM5->CNT, GPIOD, GPIO_Pin_0, GPIO_Pin_1);
	forearm.init(&TIM2->CCR3, &TIM2->CCR4, &TIM3->CNT);
	arm.init(&TIM2->CCR2, &TIM1->CNT, GPIOD, GPIO_Pin_4, GPIO_Pin_7);
	wristRotation.init(&TIM4->CCR4, &TIM8->CNT, GPIOE, GPIO_Pin_0, GPIO_Pin_4);
	wristPitch.init(1);
	gripperRotation.init(2);
	gripper.init(&TIM4->CCR3, &TIM5->CNT, GPIOE, GPIO_Pin_2, GPIO_Pin_3);
}

const uint8_t ManipulatorStateMachine::init()
{
	RCC_CONF();
	GPIO_MOTOR_CONF();
	TIM_PWM_CONF();
	USART1_CONF();
	USART2_CONF();
	return 0;
}

void ManipulatorStateMachine::setJointsVel(const double* velocities)
{
	for(int i = 0; i < NUM_JOINTS; ++i)
	{
		joints[i]->setVelocity(velocities[i]);
	}
}

void ManipulatorStateMachine::closeGripper(const double& velocity)
{
	gripper.setVelocity(velocity);
}

void ManipulatorStateMachine::openGripper(const double& velocity)
{
	gripper.setVelocity(-velocity);
}
