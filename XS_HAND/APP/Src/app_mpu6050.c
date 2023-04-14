/*
 * app_mpu6050.c
 *
 *  Created on: Apr 13, 2023
 *      Author: fsr
 */
#include "app_log.h"
#include "bsp_mpu6050.h"
#include "freertos.h"
#include "app_mpu6050.h"
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
	temp_data = Send_Data.Gx*1000;
	BUFF[_cnt++]=BYTE0(temp_data);							//数据内容,小段模式，低位在前
	BUFF[_cnt++]=BYTE1(temp_data);							//需要将字节进行拆分，调用上面的宏定义即可。
	temp_data = Send_Data.Gy*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	temp_data = Send_Data.Gz*1000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	temp_data = Send_Data.Ax*10000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	BUFF[_cnt++]=BYTE2(temp_data);
	BUFF[_cnt++]=BYTE3(temp_data);
	temp_data = Send_Data.Ay*10000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	BUFF[_cnt++]=BYTE2(temp_data);
	BUFF[_cnt++]=BYTE3(temp_data);
	temp_data = Send_Data.Az*10000;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	BUFF[_cnt++]=BYTE2(temp_data);
	BUFF[_cnt++]=BYTE3(temp_data);
	temp_data = Send_Data.KalmanAngleX*10;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	temp_data = Send_Data.KalmanAngleY*10;
	BUFF[_cnt++]=BYTE0(temp_data);
	BUFF[_cnt++]=BYTE1(temp_data);
	BUFF[_cnt++]=BYTE0(Send_Data.Gyro_Z_RAW);
	BUFF[_cnt++]=BYTE1(Send_Data.Gyro_Z_RAW);

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
