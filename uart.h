/*
 * uart.h
 *
 *  Created on: 2022. 4. 15.
 *      Author: hc.ro
 */

#ifndef UART_H_
#define UART_H_

#include <msp430.h>
#include "CircularQueue.h"
#include <stdio.h>
#include <string.h>

unsigned char RxData;
unsigned char RxData2;      // Uart2


Queue q;

void Init_UART(void);
void Init_UART2(void);


void fput_data(unsigned char _c);
void fput_data2(unsigned char _c);


// transmit as 'PRINTF' function
int fputc(int _c, register FILE *_fp);
int fputs(const char *_ptr, register FILE *_fp);

void UART_TX_DATA(void);

#endif /* UART_H_ */
