/*
 * ManipulatorStateMachine.cpp
 *
 *  Created on: Jul 11, 2019
 *      Author: mrbr
 */

#include "ManipulatorStateMachine.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"

ManipulatorStateMachine::ManipulatorStateMachine()
{
	joints[0].reset(new PWMMotor(TIM2->CCR1, TIM5->CNT, GPIOD, GPIO_Pin_0, GPIO_Pin_1));
	joints[1].reset(new HighPowerMotor(TIM2->CCR1, TIM5->CNT, GPIOD, GPIO_Pin_0, GPIO_Pin_1));
	joints[2].reset(new PWMMotor(TIM2->CCR2, TIM1->CNT, GPIOD, GPIO_Pin_4, GPIO_Pin_7));
	joints[3].reset(new PWMMotor(TIM4->CCR4, TIM8->CNT, GPIOE, GPIO_Pin_0, GPIO_Pin_4));
	joints[4].reset(new ServoMotor(1));
	joints[5].reset(new ServoMotor(2));
	gripper = PWMMotor(TIM4->CCR3, TIM5->CNT, GPIOE, GPIO_Pin_2, GPIO_Pin_3);
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

void ManipulatorStateMachine::setJointsVel(const std::vector<double>& velocities)
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
