
#ifndef _UART_H
#define _UART_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define UART_BUF_SIZE			1024

int UART1_Init(uint32_t baudrate);
void UART1_ISR(void);
void UART1_Send(uint8_t *buf, uint16_t len);
int UART1_Get(uint8_t *buf, uint32_t *len);


int UART3_Init(uint32_t baudrate);
void UART3_ISR(void);
void UART3_Send(uint8_t *buf, uint16_t len);
int UART3_Get(uint8_t *buf, uint32_t *len);




#endif	//_UART_H
