/*
 * app_mpu6050.h
 *
 *  Created on: Apr 13, 2023
 *      Author: fsr
 */

#ifndef INC_APP_MPU6050_H_
#define INC_APP_MPU6050_H_

/**********为了匿名四轴上位机的协议定义的变量****************************/
//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	 //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


void sent_mpu6050(const MPU6050_t Send_Data);


#endif /* INC_APP_MPU6050_H_ */
