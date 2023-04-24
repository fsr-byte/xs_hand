# XS_Hand

#### 介绍
使用正点原子精英板，简单控制MPU6050，一直DMP库完成
    USART
PA10      - USART1_RX
PA9       - USART1_TX

    MPU6050
PB10      - I2C2_SCL
PB11      - I2C2_SDA
PA15      - AD0
PA2       - INT

    else
PB5       - LED1
PE5       - LED2    
PA0       - KEY

使用匿名上位机进行显示

#### 问题/优化点
1.在连接上位机，上位机串口（目前只有匿名上位机会出现该问题，其他的上位机无问题）打开时，板子复位重新上电时会导致代码崩溃，猜测原因是因为上位机串口处罚了中断。
2.串口可以使用fifo收发（优化点）
3.mpu6050读取的dmp数据的滤波优化