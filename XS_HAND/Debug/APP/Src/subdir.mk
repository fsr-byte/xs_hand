################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/Src/app_6050.c \
../APP/Src/app_log.c \
../APP/Src/app_mpu6050.c \
../APP/Src/app_tasks.c \
../APP/Src/app_usart.c 

OBJS += \
./APP/Src/app_6050.o \
./APP/Src/app_log.o \
./APP/Src/app_mpu6050.o \
./APP/Src/app_tasks.o \
./APP/Src/app_usart.o 

C_DEPS += \
./APP/Src/app_6050.d \
./APP/Src/app_log.d \
./APP/Src/app_mpu6050.d \
./APP/Src/app_tasks.d \
./APP/Src/app_usart.d 


# Each subdirectory must supply rules for building sources it contributes
APP/Src/%.o APP/Src/%.su APP/Src/%.cyclo: ../APP/Src/%.c APP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL_TARGET_STM32F1 -DEMPL -DMPL_LOG_NDEBUG=0 -DMPU6050 -DUSE_DMP -DREMOVE_LOGGING -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fsr/code/xs_-hand/XS_HAND/APP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/BSP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/eMPL" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/include" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/stm32L" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/eMPL-hal" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/mllite" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/mpl" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/porting" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-APP-2f-Src

clean-APP-2f-Src:
	-$(RM) ./APP/Src/app_6050.cyclo ./APP/Src/app_6050.d ./APP/Src/app_6050.o ./APP/Src/app_6050.su ./APP/Src/app_log.cyclo ./APP/Src/app_log.d ./APP/Src/app_log.o ./APP/Src/app_log.su ./APP/Src/app_mpu6050.cyclo ./APP/Src/app_mpu6050.d ./APP/Src/app_mpu6050.o ./APP/Src/app_mpu6050.su ./APP/Src/app_tasks.cyclo ./APP/Src/app_tasks.d ./APP/Src/app_tasks.o ./APP/Src/app_tasks.su ./APP/Src/app_usart.cyclo ./APP/Src/app_usart.d ./APP/Src/app_usart.o ./APP/Src/app_usart.su

.PHONY: clean-APP-2f-Src

