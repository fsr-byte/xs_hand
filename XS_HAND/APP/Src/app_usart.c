/*
 * APP_USART.c
 *
 *  Created on: 2023年4月13日
 *      Author: fsr
 */
#include "bsp_usart.h"
#include "app_usart.h"
uint8_t send_cnt = 0;
void test_fun(uint8_t *rx_data_buf, uint8_t data_len)
{
	send_cnt++;
	UART_DMA_Send(rx_data_buf,data_len);
}


void UART_DMA_RX_Data_Analyze(uint8_t *rx_data_buf, uint8_t data_len)
{
	test_fun(rx_data_buf,data_len);
}
