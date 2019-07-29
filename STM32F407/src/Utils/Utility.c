/*
 * Utility.c
 *
 *  Created on: Jul 25, 2019
 *      Author: mrbr
 */

#include "Utility.h"

void send_char(char c, USART_TypeDef* USART)
{
    while (USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
    USART_SendData(USART, c);
}

void send_string(const char* s, USART_TypeDef* USART)
{
    while (*s)
        send_char(*s++, USART);
}

int _write(int file, char *ptr, int len)
{
	switch(file)
	{
	case STDOUT_FILENO ... STDERR_FILENO: //fall-through
		send_string(ptr, USART2);
		break;
	default:
		return -1;
	}
	return len;
}

