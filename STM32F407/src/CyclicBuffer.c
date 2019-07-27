/*
 * CyclicBuffer.c
 *
 *  Created on: Jul 26, 2019
 *      Author: mrbr
 */

#include <stddef.h>

#include "CyclicBuffer.h"

void clearBuffer(cyclicBuffer* buf)
{
	buf->end = buf->start = buf->data;
}

size_t getBufferOccupiedSize(cyclicBuffer* buf)
{
	if (buf->end >= buf->start)
		return (size_t)(buf->end - buf->start);
	else
		return (size_t)(BUF_MAX_LEN - (buf->start - buf->end));
}

uint8_t getBufferByte (cyclicBuffer* buf)
{	uint8_t byte = *(buf->start);
	++(buf->start);
    if (buf->start >= buf->data + BUF_MAX_LEN)
    	buf->start = buf->data;

    return byte;
}

uint8_t peekBufferByte(cyclicBuffer* buf, uint8_t idx)
{
	return *(buf->start+idx);
}

void setBufferByte (cyclicBuffer* buf, uint8_t byte)
{
	*(buf->end) = byte;
	++(buf->end);
    if (buf->end >= buf->end + BUF_MAX_LEN)
    	buf->end = buf->data;
}

