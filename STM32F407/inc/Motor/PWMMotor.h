/*
 * PWMMotor.h
 *
 *  Created on: Jul 16, 2019
 *      Author: mrbr
 */

#ifndef SRC_PWMMOTOR_H_
#define SRC_PWMMOTOR_H_

#include "Motor.h"
#include "stm32f4xx_gpio.h"

class PWMMotor: public Motor {
public:
	PWMMotor():
		pwmChannel(nullptr),
		gpioPort(nullptr),
		pinCW(0),
		pinCCW(0){};

	PWMMotor(volatile uint32_t * pwm_channel,
			volatile uint32_t * encoder_cnt,
			GPIO_TypeDef * port, const uint16_t& pin_1,
			const uint16_t& pin_2);

	virtual ~PWMMotor(){};

	void init(volatile uint32_t * pwm_channel,
			volatile uint32_t * encoder_cnt,
			GPIO_TypeDef * port, const uint16_t& pin_1,
			const uint16_t& pin_2);

	virtual void setVelocity(const double& velocity);

private:
	volatile uint32_t * pwmChannel;
	GPIO_TypeDef * gpioPort;
	uint16_t pinCW;
	uint16_t pinCCW;
};

#endif /* SRC_PWMMOTOR_H_ */
