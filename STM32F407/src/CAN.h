#ifndef MANIPULATOR_CAN_H_
#define MANIPULATOR_CAN_H_

#include "stm32f4xx.h"

CanTxMsg txMessage;
CanRxMsg rxMessage;

void initCan(void);
void CAN1_RX0_IRQHandler(void);

void CAN_send_data(void);

#endif /* MANIPULATOR_CAN_H_ */
