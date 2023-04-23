/*
 * app_mpu6050.c
 *
 *  Created on: Apr 13, 2023
 *      Author: fsr
 */
#include "bsp_mpu6050.h"
#include "bsp_usart.h"

/**********为了匿名四轴上位机的协议定义的变量****************************/
//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	 //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
/*
*	使用匿名上位机发送
 *
 */
uint8_t BUFF[100];
void sent_mpu6050(const MPU6050_t Send_Data)
{
	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt=0;
	int32_t temp_data;
	BUFF[_cnt++]=0xAA;//帧头
	BUFF[_cnt++]=0xFF;//目标地址
	BUFF[_cnt++]=0XF1;//功能码
	BUFF[_cnt++]=0x00;//数据长度,先保留，等长度确定
	temp_data = Send_Data.Ax*1000;
	BUFF[_cnt++]=BYTE0(temp_data);							//数据内容,小段模式，低位在前
	BUFF[_cnt++]=BYTE1(temp_data);							//需要将字节进行拆分，调用上面的宏定义即可。
	temp_data = Send_Data.Ay*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	temp_data = Send_Data.Az*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	temp_data = Send_Data.Gx*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	BUFF[_cnt++]=BYTE2(temp_data);
	BUFF[_cnt++]=BYTE3(temp_data);
	temp_data = Send_Data.Gy*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	BUFF[_cnt++]=BYTE2(temp_data);
	BUFF[_cnt++]=BYTE3(temp_data);
	temp_data = Send_Data.Gz*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	BUFF[_cnt++]=BYTE2(temp_data);
	BUFF[_cnt++]=BYTE3(temp_data);
	temp_data = Send_Data.Accel_X*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	temp_data = Send_Data.Accel_Y*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	temp_data = Send_Data.Accel_Z*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
//	BUFF[_cnt++]=BYTE0(Send_Data.Gyro_Z_RAW);
//	BUFF[_cnt++]=BYTE1(Send_Data.Gyro_Z_RAW);

	BUFF[3]=_cnt-4;			//数据长度
	//SC和AC的校验直接抄最上面上面简介的即可
	for(i=0;i<BUFF[3]+4;i++)
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;
	BUFF[_cnt++]=addcheck;
	UART_DMA_Send(BUFF,++_cnt);
//	for(i=0;i<_cnt;i++) UsartSendByte(USART1,BUFF[i]);//串口逐个发送数据
}
