#include <stdio.h>
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "bsp_usart.h"

uint8_t UART_Rx_Len;
uint8_t UART_Rx_Data[UART_DMA_BUFF_SIZE];

__weak void UART_DMA_RX_Data_Analyze(uint8_t *rx_data_buf, uint8_t data_len)
{

}

void UART_DMA_Recive_Init(void)
{
    //使能接收，进入中断回调函数
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_Rx_Data, UART_DMA_BUFF_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    UNUSED(Size);
    if (huart->Instance == USART1)
    {
		UART_Rx_Len = Size;
        UART_DMA_RX_Data_Analyze(UART_Rx_Data, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART_Rx_Data, UART_DMA_BUFF_SIZE);
    }
    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UARTEx_RxEventCallback can be implemented in the user file.
     */
}

void UART_DMA_Send(uint8_t *pData, uint16_t Size)
{
    HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

/*************************************************
  * 函数功能: 串口发送错误进行串口复位操作
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  **********************************************/
void UART_REST(UART_HandleTypeDef *uartHandle)
{
    if (__HAL_UART_GET_FLAG(uartHandle, UART_FLAG_ORE) != RESET) //串口复位
    {
        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_ORE);//退出中断，并清除错误标志位
        __HAL_UART_CLEAR_OREFLAG(uartHandle);
        HAL_UART_MspDeInit(uartHandle);
        HAL_UART_MspInit(uartHandle);
        if (uartHandle->Instance == USART1)
        {
            HAL_UART_Receive_DMA(uartHandle, UART_Rx_Data, UART_DMA_BUFF_SIZE);	 // 这里加这个函数，接收数据
        }
    }
    if (__HAL_UART_GET_FLAG(uartHandle, UART_FLAG_PE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_PE);//退出中断，并清除错误标志位
        HAL_UART_MspDeInit(uartHandle);
        HAL_UART_MspInit(uartHandle);
        if (uartHandle->Instance == USART1)
        {
            HAL_UART_Receive_DMA(uartHandle, UART_Rx_Data, UART_DMA_BUFF_SIZE);	 // 这里加这个函数，接收数据
        }
    }
    if (__HAL_UART_GET_FLAG(uartHandle, UART_FLAG_FE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(uartHandle, UART_FLAG_FE);//退出中断，并清除错误标志位
        HAL_UART_MspDeInit(uartHandle);
        HAL_UART_MspInit(uartHandle);
        if (uartHandle->Instance == USART1)
        {
            HAL_UART_Receive_DMA(uartHandle, UART_Rx_Data, UART_DMA_BUFF_SIZE);	 // 这里加这个函数，接收数据
        }
    }
}

/**
  * 函数功能: 重定向c库函数printf到huart2
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int _write(int file, char *data, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }

   // arbitrary timeout 1000
   HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);

   // return # of bytes written - as best we can tell
   return (status == HAL_OK ? len : 0);
}

///**
//  * 函数功能: 重定向c库函数printf到huart2
//  * 输入参数: 无
//  * 返 回 值: 无
//  * 说    明：无
//  */
//int fputc(int ch, FILE *f)
//{
//    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
//    return ch;
//}


