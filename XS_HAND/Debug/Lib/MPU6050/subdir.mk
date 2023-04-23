################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/MPU6050/mpu6050.c 

OBJS += \
./Lib/MPU6050/mpu6050.o 

C_DEPS += \
./Lib/MPU6050/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/MPU6050/%.o Lib/MPU6050/%.su Lib/MPU6050/%.cyclo: ../Lib/MPU6050/%.c Lib/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fsr/code/xs_-hand/XS_HAND/APP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/BSP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Lib/MPU6050" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Lib/MPU6050/eMPL" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lib-2f-MPU6050

clean-Lib-2f-MPU6050:
	-$(RM) ./Lib/MPU6050/mpu6050.cyclo ./Lib/MPU6050/mpu6050.d ./Lib/MPU6050/mpu6050.o ./Lib/MPU6050/mpu6050.su

.PHONY: clean-Lib-2f-MPU6050

