#ifndef MANIPULATOR_CAN_H_
#define MANIPULATOR_CAN_H_

#include "stm32f4xx_can.h"

CanTxMsg txMessage;
CanRxMsg rxMessage;

#ifdef __cplusplus
extern "C" {
#endif

void CAN1_RX0_IRQHandler(void);
void CAN_send_data(void);

#ifdef __cplusplus
}
#endif

#endif /* MANIPULATOR_CAN_H_ */
