################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/MPU6050/eMPL/inv_mpu.c \
../Lib/MPU6050/eMPL/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Lib/MPU6050/eMPL/inv_mpu.o \
./Lib/MPU6050/eMPL/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Lib/MPU6050/eMPL/inv_mpu.d \
./Lib/MPU6050/eMPL/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/MPU6050/eMPL/%.o Lib/MPU6050/eMPL/%.su Lib/MPU6050/eMPL/%.cyclo: ../Lib/MPU6050/eMPL/%.c Lib/MPU6050/eMPL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fsr/code/xs_-hand/XS_HAND/APP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/BSP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Lib/MPU6050" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Lib/MPU6050/eMPL" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lib-2f-MPU6050-2f-eMPL

clean-Lib-2f-MPU6050-2f-eMPL:
	-$(RM) ./Lib/MPU6050/eMPL/inv_mpu.cyclo ./Lib/MPU6050/eMPL/inv_mpu.d ./Lib/MPU6050/eMPL/inv_mpu.o ./Lib/MPU6050/eMPL/inv_mpu.su ./Lib/MPU6050/eMPL/inv_mpu_dmp_motion_driver.cyclo ./Lib/MPU6050/eMPL/inv_mpu_dmp_motion_driver.d ./Lib/MPU6050/eMPL/inv_mpu_dmp_motion_driver.o ./Lib/MPU6050/eMPL/inv_mpu_dmp_motion_driver.su

.PHONY: clean-Lib-2f-MPU6050-2f-eMPL

