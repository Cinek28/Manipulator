#include "CAN.h"

// Sample function sending data:
void CAN_send_data(void) {
	txMessage.StdId = 120;  // frame id
	txMessage.RTR = CAN_RTR_DATA;
	txMessage.IDE = CAN_ID_STD;
	txMessage.DLC = 6;      // frame length
	txMessage.Data[0] = 'a'; // data send
	txMessage.Data[1] = 'b';
	txMessage.Data[2] = (uint8_t)105;
	txMessage.Data[3] = 25;
	txMessage.Data[4] = 0x54;
	txMessage.Data[5] = 'F';
	CAN_Transmit(CAN1, &txMessage);
}

void CAN1_RX0_IRQHandler(void) {
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rxMessage);
		switch (rxMessage.StdId) {
		//TODO:
		default:
			break;
		}
	}
}
