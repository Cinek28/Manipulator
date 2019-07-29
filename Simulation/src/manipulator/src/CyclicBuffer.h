/*
 * CyclicBuffer.h
 *
 *  Created on: Jul 26, 2019
 *      Author: mrbr
 */

#ifndef CYCLICBUFFER_H_
#define CYCLICBUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BUF_MAX_LEN 32

typedef struct cyclicBuffer
{
	volatile uint8_t data[BUF_MAX_LEN];
	volatile uint8_t* volatile end;
	volatile uint8_t* volatile start;
} cyclicBuffer;

void clearBuffer(cyclicBuffer* buf);
size_t getBufferOccupiedSize(cyclicBuffer* buf);
uint8_t getBufferByte (cyclicBuffer* buf);
void setBufferByte (cyclicBuffer* buf, uint8_t byte);
uint8_t peekBufferByte(cyclicBuffer* buf, uint8_t idx);


#ifdef __cplusplus
}
#endif

#endif /* CYCLICBUFFER_H_ */
