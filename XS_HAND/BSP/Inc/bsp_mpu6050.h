/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "i2c.h"

// MPU6050 structure
typedef struct
{
    int16_t Accel_X_RAW;   //X轴加速度计的原始数据
    int16_t Accel_Y_RAW;   //Y轴加速度计的原始数据
    int16_t Accel_Z_RAW;   //Z轴加速度计的原始数据
    double Ax;             //X轴加速度值
    double Ay;             //Y轴加速度值
    double Az;             //Z轴加速度值

    double Accel_X;		   //去除重力后的X轴加速度
    double Accel_Y;		   //去除重力后的Y轴加速度
    double Accel_Z;		   //去除重力后的Z轴加速度

    int16_t Gyro_X_RAW;    //X轴陀螺仪的原始数据
    int16_t Gyro_Y_RAW;    //Y轴陀螺仪的原始数据
    int16_t Gyro_Z_RAW;    //Z轴陀螺仪的原始数据
    double Gx;             //X轴角速度值
    double Gy;             //Y轴角速度值
    double Gz;             //Z轴角速度值

    float Temperature;     //温度值

    double KalmanAngleX;   //卡尔曼滤波后的 X 轴角度值
    double KalmanAngleY;   //卡尔曼滤波后的 Y 轴角度值
} MPU6050_t;


// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

void remove_gravity(MPU6050_t *DataStruct);
