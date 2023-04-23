/*
 * app_6050.c
 *
 *  Created on: 2023年4月23日
 *      Author: fsr
 */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "global_var.h"
#include "app_mpu6050.h"
#include "bsp_usart.h"
#include "bsp_mpu6050.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define PRINT_INSTAND_SWJ
#include "STM32F1_porting.h"
#include "mpu6050_SL.h"

#include "stdio.h"

/* mpu6050 DMP库  begin */
#include "eMPL_outputs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "log.h"
#include "mltypes.h"
#include "mpu.h"
#include "packet.h"

extern MPU6050_t MPU6050_Data;

extern void get_ms_user(unsigned long *count); //定义在inv_mpu.c
#define get_tick_count get_ms_user
/* mpu6050 DMP库  end */
/* USER CODE BEGIN PD */
/* Data read from MPL. */
#define PRINT_ACCEL (0x01)
#define PRINT_GYRO (0x02)
#define PRINT_QUAT (0x04)
#define PRINT_COMPASS (0x08)
#define PRINT_EULER (0x10)
#define PRINT_ROT_MAT (0x20)
#define PRINT_HEADING (0x40)
#define PRINT_PEDO (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;

/* USER CODE END PD */

/**
 * @brief  控制串口发送1个字符
 * @param  c:要发送的字符
 * @retval none
 */
void usart_send_char(uint8_t c) {
//	HAL_UART_Transmit(&huart1, &c, 1);
	// while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET)
	//     ; //循环发送,直到发送完毕
	// USART_SendData(DEBUG_USARTx, c);
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

/*函数功能：根据匿名最新上位机协议写的显示姿态的程序（上位机0512版本）
 *具体协议说明请查看上位机软件的帮助说明。
 */
void Data_Send_Status(float Pitch, float Roll, float Yaw) {

#ifdef PRINT_INSTAND_SWJ
    printf("PIT:%4.1f  ROL:%4.1f  YAW:%4.1f  \n", Pitch, Roll, Yaw);
#else
	unsigned char i = 0;
	unsigned char _cnt = 0, sum = 0;
	unsigned int _temp;
	uint8_t data_to_send[50];
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x01;
	data_to_send[_cnt++] = 0;

	_temp = (int) (Roll * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = 0 - (int) (Pitch * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int) (Yaw * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = 0;
	data_to_send[_cnt++] = BYTE3(_temp);
	data_to_send[_cnt++] = BYTE2(_temp);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[_cnt++] = 0xA0;

	data_to_send[3] = _cnt - 4;
	//和校验
	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

//	//串口发送数据
//	for (i = 0; i < _cnt; i++)
//		usart_send_char(data_to_send[i]);
#endif
}

/*函数功能：根据匿名最新上位机协议写的显示传感器数据（上位机0512版本）
 *具体协议说明请查看上位机软件的帮助说明。
 */
void Send_Data(int16_t *Gyro, int16_t *Accel) {

#ifdef PRINT_INSTAND_SWJ
    // printf("加速度: X%5d\tY%5d\tZ%5d || 角加速度: X%5d\tY%5d\tZ%5d\n", Accel[0], Accel[1], Accel[2], Gyro[0],
    // Gyro[1], Gyro[2]);
#else
	unsigned char i = 0;
	unsigned char _cnt = 0, sum = 0;
	//	unsigned int _temp;
	uint8_t data_to_send[50];
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x02;
	data_to_send[_cnt++] = 0;

	data_to_send[_cnt++] = BYTE1(Accel[0]);
	data_to_send[_cnt++] = BYTE0(Accel[0]);
	data_to_send[_cnt++] = BYTE1(Accel[1]);
	data_to_send[_cnt++] = BYTE0(Accel[1]);
	data_to_send[_cnt++] = BYTE1(Accel[2]);
	data_to_send[_cnt++] = BYTE0(Accel[2]);

	data_to_send[_cnt++] = BYTE1(Gyro[0]);
	data_to_send[_cnt++] = BYTE0(Gyro[0]);
	data_to_send[_cnt++] = BYTE1(Gyro[1]);
	data_to_send[_cnt++] = BYTE0(Gyro[1]);
	data_to_send[_cnt++] = BYTE1(Gyro[2]);
	data_to_send[_cnt++] = BYTE0(Gyro[2]);
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;

	data_to_send[3] = _cnt - 4;
	//和校验
	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	//串口发送数据
//	for (i = 0; i < _cnt; i++)
//		usart_send_char(data_to_send[i]);
#endif
}

extern struct inv_sensor_cal_t sensors;

/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
static void read_from_mpl(void) {
	long msg, data[9];
	int8_t accuracy;
	unsigned long timestamp;
	float float_data[3] = { 0 };

	//MPU_DEBUG_FUNC();
	if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*) &timestamp)) {
		/* Sends a quaternion packet to the PC. Since this is used by the Python
		 * test app to visually represent a 3D quaternion, it's sent each time
		 * the MPL has new data.
		 */
		eMPL_send_quat(data);

		/* Specific data packets can be sent or suppressed using USB commands. */
		if (hal.report & PRINT_QUAT)
			eMPL_send_data(PACKET_DATA_QUAT, data);
	}
		//加速度计数据
	if (hal.report & PRINT_ACCEL) {
		if (inv_get_sensor_type_accel(data, &accuracy,
				(inv_time_t*) &timestamp))
		MPU6050_Data.Ax = data[0] * 1.0 / (1 << 16);
		MPU6050_Data.Ay = data[1] * 1.0 / (1 << 16);
		MPU6050_Data.Az = data[2] * 1.0 / (1 << 16);
			eMPL_send_data(PACKET_DATA_ACCEL, data);
	}
		//陀螺仪数据
	if (hal.report & PRINT_GYRO) {
		if (inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_GYRO, data);
	}
#ifdef COMPASS_ENABLED
    if (hal.report & PRINT_COMPASS)
    {
        if (inv_get_sensor_type_compass(data, &accuracy, (inv_time_t *)&timestamp))
            eMPL_send_data(PACKET_DATA_COMPASS, data);
    }
#endif
    //欧拉角数据
	if (hal.report & PRINT_EULER) {
		if (inv_get_sensor_type_euler(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_EULER, data);
	}
	/*********发送数据到匿名四轴上位机**********/
	if (1) {
#ifdef USE_LCD_DISPLAY

        char cStr[70];

#endif

		unsigned long timestamp, step_count, walk_time;

		/*获取欧拉角*/
		if (inv_get_sensor_type_euler(data, &accuracy,
				(inv_time_t*) &timestamp)) {
			float Pitch, Roll, Yaw;
			Pitch = data[0] * 1.0 / (1 << 16);
			Roll = data[1] * 1.0 / (1 << 16);
			Yaw = data[2] * 1.0 / (1 << 16);


			/*向匿名上位机发送姿态*/
//			MPU6050_Data.Gx = Pitch;
//			MPU6050_Data.Gy = Roll;
//			MPU6050_Data.Gz = Yaw;

			//					Data_Send_Status(Pitch, Roll, Yaw);
			//					/*向匿名上位机发送原始数据*/
			//					Send_Data((int16_t*) &sensors.gyro.raw,
			//							(int16_t*) &sensors.accel.raw);
			//		MPU6050_Read_All(&hi2c2, &MPU6050_Data);

//			/*向匿名上位机发送姿态*/
//			Data_Send_Status(Pitch, Roll, Yaw);
//			/*向匿名上位机发送原始数据*/
//			Send_Data((int16_t*) &sensors.gyro.raw,
//					(int16_t*) &sensors.accel.raw);

#ifdef USE_LCD_DISPLAY

            sprintf(cStr, "Pitch :   %.4f   ", Pitch); // inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
            ILI9341_DispString_EN(30, 90, cStr);

            sprintf(cStr, "Roll :   %.4f   ", Roll); // inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
            ILI9341_DispString_EN(30, 110, cStr);

            sprintf(cStr, "Yaw :   %.4f   ", Yaw); // inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
            ILI9341_DispString_EN(30, 130, cStr);

            /*温度*/
            mpu_get_temperature(data, (inv_time_t *)&timestamp);

            sprintf(cStr, "Temperature :   %.2f   ",
                    data[0] * 1.0 / (1 << 16)); // inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
            ILI9341_DispString_EN(30, 150, cStr);
#endif
		}

		/*获取步数*/
		get_tick_count(&timestamp);
		if (timestamp > hal.next_pedo_ms) {

			hal.next_pedo_ms = timestamp + PEDO_READ_MS;
			dmp_get_pedometer_step_count(&step_count);
			dmp_get_pedometer_walk_time(&walk_time);

#ifdef USE_LCD_DISPLAY

            sprintf(cStr, "Walked steps :  %ld  steps over  %ld  milliseconds..", step_count, walk_time);

            ILI9341_DispString_EN(0, 180, cStr);
#endif
		}
	}

	if (hal.report & PRINT_ROT_MAT) {
		if (inv_get_sensor_type_rot_mat(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_ROT, data);
	}
	if (hal.report & PRINT_HEADING) {
		if (inv_get_sensor_type_heading(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_HEADING, data);
	}
	if (hal.report & PRINT_LINEAR_ACCEL) {
		if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy,
				(inv_time_t*) &timestamp)) {
			MPU6050_Data.Gx = data[0];
			MPU6050_Data.Gy = data[1];
			MPU6050_Data.Gz = data[2];
			MPL_LOGI("Linear Accel: %7.5f\t %7.5f\t %7.5f\t\r\n", float_data[0],
					float_data[1], float_data[2]);
		}
	}
	if (hal.report & PRINT_GRAVITY_VECTOR) {
		if (inv_get_sensor_type_gravity(float_data, &accuracy,
				(inv_time_t*) &timestamp))
			MPU6050_Data.Accel_X = data[0];
			MPU6050_Data.Accel_Y = data[1];
			MPU6050_Data.Accel_Z = data[2];
			MPL_LOGI("Gravity Vector: %7.5f\t %7.5f\t %7.5f\t\r\n",
					float_data[0], float_data[1], float_data[2]);
	}
	if (hal.report & PRINT_PEDO) {
		unsigned long timestamp;
		get_tick_count(&timestamp);
		if (timestamp > hal.next_pedo_ms) {
			hal.next_pedo_ms = timestamp + PEDO_READ_MS;
			unsigned long step_count, walk_time;
			dmp_get_pedometer_step_count(&step_count);
			dmp_get_pedometer_walk_time(&walk_time);
			MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
					walk_time);
		}
	}

	/* Whenever the MPL detects a change in motion state, the application can
	 * be notified. For this example, we use an LED to represent the current
	 * motion state.
	 */
	msg = inv_get_message_level_0(
			INV_MSG_MOTION_EVENT | INV_MSG_NO_MOTION_EVENT);
	if (msg) {
		if (msg & INV_MSG_MOTION_EVENT) {
			MPL_LOGI("Motion!\n");
		} else if (msg & INV_MSG_NO_MOTION_EVENT) {
			MPL_LOGI("No motion!\n");
		}
	}
	sent_mpu6050(MPU6050_Data);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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

	static uint16_t Run_Times ;
	static unsigned char new_temp = 0;
	static unsigned long timestamp;
	static unsigned long sensor_timestamp;
	static int new_data = 0;
	MPU6050_mpu_init();
	MPU6050_mpl_init();
	MPU6050_config();
	hal.sensors ^= ACCEL_ON;
	hal.report ^= PRINT_ACCEL;
//	hal.report ^= PRINT_LINEAR_ACCEL;
//	hal.report ^= PRINT_GRAVITY_VECTOR;
	while (1) {
		Run_Times++;

		if(Run_Times % 500 == 0)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		}
		osDelay(1);
//		if(Run_Times % 20 == 0)
//		{
//			MPU6050_data_ready_cb();
//		}
		if (!hal.sensors || !hal.new_gyro) {
			continue;
		}

		// if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
		// // __HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)
		// {
		//     /* A byte has been received via USART. See handle_input for a list of
		//      * valid commands.
		//      */

		//     // USART_ClearFlag(DEBUG_USARTx, USART_FLAG_RXNE);
		//     __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);

		//     // handle_input();
		// }

#ifdef PRINT_INSTAND_SWJ
		osDelay(500);
#endif

		get_tick_count(&timestamp);
		if (timestamp > hal.next_temp_ms) {
			hal.next_temp_ms = timestamp + TEMP_READ_MS;
			new_temp = 1;
		}

		if (hal.motion_int_mode) {
			/* Enable motion interrupt. */
			mpu_lp_motion_interrupt(500, 1, 5);
			/* Notify the MPL that contiguity was broken. */
			inv_accel_was_turned_off();
			inv_gyro_was_turned_off();
			inv_compass_was_turned_off();
			inv_quaternion_sensor_was_turned_off();
			/* Wait for the MPU interrupt. */
			while (!hal.new_gyro) {
			}
			/* Restore the previous sensor configuration. */
			mpu_lp_motion_interrupt(0, 0, 0);
			hal.motion_int_mode = 0;
		}

		if (hal.new_gyro && hal.dmp_on) {
			short gyro[3], accel_short[3], sensors;
			unsigned char more;
			long accel[3], quat[4], temperature;
			/* This function gets new data from the FIFO when the DMP is in
			 * use. The FIFO can contain any combination of gyro, accel,
			 * quaternion, and gesture data. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
			 * the FIFO isn't being filled with accel data.
			 * The driver parses the gesture data to determine if a gesture
			 * event has occurred; on an event, the application will be notified
			 * via a callback (assuming that a callback function was properly
			 * registered). The more parameter is non-zero if there are
			 * leftover packets in the FIFO.
			 */
			dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,
					&more);
			if (!more)
				hal.new_gyro = 0;

			if (sensors & INV_WXYZ_QUAT) {

				inv_build_quat(quat, 0, sensor_timestamp);
				new_data = 1;
			}

			if (sensors & INV_XYZ_GYRO) {
				/* Push the new data to the MPL. */
				inv_build_gyro(gyro, sensor_timestamp);
				new_data = 1;
				if (new_temp) {
					new_temp = 0;
					/* Temperature only used for gyro temp comp. */
					mpu_get_temperature(&temperature, &sensor_timestamp);
					inv_build_temp(temperature, sensor_timestamp);
				}
			}
			if (sensors & INV_XYZ_ACCEL) {
				accel[0] = (long) accel_short[0];
				accel[1] = (long) accel_short[1];
				accel[2] = (long) accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
			}
		}

#if 0
            if (new_data)
            {
                long data[9];
                int8_t accuracy;
                if (inv_execute_on_data())
                {
                    printf("ERROR execute on data\n");
                }
                /* This function reads bias-compensated sensor data and sensor
                 * fusion outputs from the MPL. The outputs are formatted as seen
                 * in eMPL_outputs.c. This function only needs to be called at the
                 * rate requested by the host.
                 */

                float float_data[3];
                printf("\r\n\r\n以下是eMPL_outputs.c中的函数获取的数据\r\n");
                if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp))
                {
                    float_data[0] = data[0] * 1.0 / (1 << 16);
                    float_data[1] = data[1] * 1.0 / (1 << 16);
                    float_data[2] = data[2] * 1.0 / (1 << 16);
                    printf("欧拉角(rad)\t\t: %7.5f\t %7.5f\t %7.5f\t\r\n", float_data[0], float_data[1], float_data[2]);
                }
                if (inv_get_sensor_type_accel(data, &accuracy, (inv_time_t *)&timestamp))
                {
                    float_data[0] = data[0] * 1.0 / (1 << 16);
                    float_data[1] = data[1] * 1.0 / (1 << 16);
                    float_data[2] = data[2] * 1.0 / (1 << 16);
                    printf("加速度(g/s2)\t\t: %7.5f\t %7.5f\t %7.5f\t\r\n", float_data[0], float_data[1], float_data[2]);
                }

                if (inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t *)&timestamp))
                {
                    float_data[0] = data[0] * 1.0 / (1 << 16);
                    float_data[1] = data[1] * 1.0 / (1 << 16);
                    float_data[2] = data[2] * 1.0 / (1 << 16);
                    printf("角速度(rad/s)\t\t: %7.5f\t %7.5f\t %7.5f\t\r\n", float_data[0], float_data[1], float_data[2]);
                }
                printf("\r\n\r\n以下是hal_outputs.c中的函数获取的数据\r\n");
                // if (inv_get_sensor_type_orientation(float_data, &accuracy, (inv_time_t *)&timestamp))
                // {
                //     printf("方位角(rad/s)\t\t: %7.5f\t %7.5f\t %7.5f\t\r\n", float_data[0], float_data[1], float_data[2]);
                // }
                // MPU9xxxxx有方位角，MPU6xxx没有磁场三轴，自然也分不清方向，所赐这里输出的数据
                inv_get_sensor_type_orientation(float_data, &accuracy, (inv_time_t *)&timestamp);
                printf("伪方位角(rad/s)\t: %7.5f\t %7.5f\t %7.5f\t 没有指南针，和欧拉角没什么区别\r\n", float_data[0],
                       float_data[1], float_data[2]);

                if (inv_get_sensor_type_accelerometer(float_data, &accuracy, (inv_time_t *)&timestamp))
                {
                    printf("加速度(m/s2)\t\t: %7.5f\t %7.5f\t %7.5f\t\r\n", float_data[0], float_data[1], float_data[2]);
                }
                if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t *)&timestamp))
                {
                    printf("线性加速度(m/s2)\t: %7.5f\t %7.5f\t %7.5f\t\r\n", float_data[0], float_data[1], float_data[2]);
                }
                if (inv_get_sensor_type_gyroscope(float_data, &accuracy, (inv_time_t *)&timestamp))
                {
                    printf("校准后角速度(rad/s)\t: %7.5f\t %7.5f\t %7.5f\t\r\n", float_data[0], float_data[1], float_data[2]);
                }
                osDelay(100);
            }

    #else
		if (new_data) {
			inv_execute_on_data();

			/* This function reads bias-compensated sensor data and sensor
			 * fusion outputs from the MPL. The outputs are formatted as seen
			 * in eMPL_outputs.c. This function only needs to be called at the
			 * rate requested by the host.
			 */
			read_from_mpl();
		}
#endif
	}

	for (;;) {


//		osDelay(10);
	}
	/* USER CODE END StartTask03 */
}
