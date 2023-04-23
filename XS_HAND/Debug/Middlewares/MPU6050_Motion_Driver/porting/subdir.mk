################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/MPU6050_Motion_Driver/porting/STM32F1_porting.c \
../Middlewares/MPU6050_Motion_Driver/porting/mpu6050_SL.c 

OBJS += \
./Middlewares/MPU6050_Motion_Driver/porting/STM32F1_porting.o \
./Middlewares/MPU6050_Motion_Driver/porting/mpu6050_SL.o 

C_DEPS += \
./Middlewares/MPU6050_Motion_Driver/porting/STM32F1_porting.d \
./Middlewares/MPU6050_Motion_Driver/porting/mpu6050_SL.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/MPU6050_Motion_Driver/porting/%.o Middlewares/MPU6050_Motion_Driver/porting/%.su Middlewares/MPU6050_Motion_Driver/porting/%.cyclo: ../Middlewares/MPU6050_Motion_Driver/porting/%.c Middlewares/MPU6050_Motion_Driver/porting/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL_TARGET_STM32F1 -DEMPL -DMPL_LOG_NDEBUG=0 -DMPU6050 -DUSE_DMP -DREMOVE_LOGGING -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fsr/code/xs_-hand/XS_HAND/APP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/BSP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/eMPL" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/include" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/stm32L" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/eMPL-hal" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/mllite" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/mpl" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/porting" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-MPU6050_Motion_Driver-2f-porting

clean-Middlewares-2f-MPU6050_Motion_Driver-2f-porting:
	-$(RM) ./Middlewares/MPU6050_Motion_Driver/porting/STM32F1_porting.cyclo ./Middlewares/MPU6050_Motion_Driver/porting/STM32F1_porting.d ./Middlewares/MPU6050_Motion_Driver/porting/STM32F1_porting.o ./Middlewares/MPU6050_Motion_Driver/porting/STM32F1_porting.su ./Middlewares/MPU6050_Motion_Driver/porting/mpu6050_SL.cyclo ./Middlewares/MPU6050_Motion_Driver/porting/mpu6050_SL.d ./Middlewares/MPU6050_Motion_Driver/porting/mpu6050_SL.o ./Middlewares/MPU6050_Motion_Driver/porting/mpu6050_SL.su

.PHONY: clean-Middlewares-2f-MPU6050_Motion_Driver-2f-porting

