#ifndef __bsp_usart_H
#define __bsp_usart_H

#include "usart.h"

#define UART_DMA_BUFF_SIZE      64          // 串口DMA发送缓冲区大小


extern uint8_t UART_Rx_Len;
extern uint8_t UART_Rx_Data[UART_DMA_BUFF_SIZE];

void UART_REST(UART_HandleTypeDef *uartHandle);
void UART_DMA_Recive_Init(void);
void UART_DMA_Send(uint8_t *pData, uint16_t Size);

#endif

