/*
 * Utility.h
 *
 *  Created on: Jul 23, 2019
 *      Author: mrbr
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include "stm32f4xx_usart.h"
//Sending data over USART:

// Redirecting streams to USART:
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

#ifdef __cplusplus
extern "C" {
#endif

void send_char(char c, USART_TypeDef* USART);
void send_string(const char* s, USART_TypeDef* USART);
int _write(int file, char *ptr, int len);

#ifdef __cplusplus
}
#endif

#endif /* UTILITY_H_ */
