/*
 * uart.h
 *
 *  Created on: Jul 18, 2023
 *      Author: picman
 */

#ifndef INC_UART_H_
#define INC_UART_H_

// include
#include "stm32f7xx.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// define
#define UART_BUFFER_SIZE 256
#define UART_TIMEOUT 100
#define NEW_LINE  "\r\n"
#define RETURN_CHAR '\r'

// functions
void sputc(char c);
void sputs(char *str);
void sputsln(char *str);
int sputsf(char *str, ...);

char sgetc(void);
int sgets(char *buffer, int len);

void startRecv();

#endif /* INC_UART_H_ */
