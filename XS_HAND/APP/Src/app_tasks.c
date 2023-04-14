/*
 * tasks.c
 *
 *  Created on: 2023年4月13日
 *      Author: fsr
 */
#include "FreeRTOS.h"
#include "global_var.h"
#include "app_log.h"
#include "bsp_usart.h"
#include "bsp_mpu6050.h"

uint8_t i = 0;
MPU6050_t MPU6050;

void Sys_Init()
{
	UART_DMA_Recive_Init();
}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask *
	 * /
	/* Infinite loop */
	Sys_Init();
	while (MPU6050_Init(&hi2c2) == 1);
	for (;;) {
		i++;
		//printf("Ax:%d,Ay:%d,Az:%d\n",MPU6050.Accel_X_RAW,MPU6050.Accel_Y_RAW,MPU6050.Accel_Z_RAW);
//		printf("Gx:%f,Gy:%f,Gz:%f\n",MPU6050.Gx,MPU6050.Gy,MPU6050.Gz);
//		sent_mpu6050(MPU6050);
		//printf("AngleX:%lf,AngleY:%lf\n",MPU6050.KalmanAngleX,MPU6050.KalmanAngleY);
		osDelay(100);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument) {
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for (;;) {
//		MPU6050_Read_All(&hi2c2, &MPU6050);
		osDelay(100);
	}
	/* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument) {
	/* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for (;;) {
		MPU6050_Read_All(&hi2c2, &MPU6050);
		sent_mpu6050(MPU6050);
		osDelay(10);
	}
	/* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument) {
	/* USER CODE BEGIN StartTask04 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartTask04 */
}
