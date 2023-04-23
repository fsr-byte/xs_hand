################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/Src/bsp_mpu6050.c \
../BSP/Src/bsp_usart.c 

OBJS += \
./BSP/Src/bsp_mpu6050.o \
./BSP/Src/bsp_usart.o 

C_DEPS += \
./BSP/Src/bsp_mpu6050.d \
./BSP/Src/bsp_usart.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/Src/%.o BSP/Src/%.su BSP/Src/%.cyclo: ../BSP/Src/%.c BSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL_TARGET_STM32F1 -DEMPL -DMPL_LOG_NDEBUG=0 -DMPU6050 -DUSE_DMP -DREMOVE_LOGGING -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fsr/code/xs_-hand/XS_HAND/APP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/BSP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/eMPL" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/include" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/stm32L" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/eMPL-hal" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/mllite" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/mpl" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/porting" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP-2f-Src

clean-BSP-2f-Src:
	-$(RM) ./BSP/Src/bsp_mpu6050.cyclo ./BSP/Src/bsp_mpu6050.d ./BSP/Src/bsp_mpu6050.o ./BSP/Src/bsp_mpu6050.su ./BSP/Src/bsp_usart.cyclo ./BSP/Src/bsp_usart.d ./BSP/Src/bsp_usart.o ./BSP/Src/bsp_usart.su

.PHONY: clean-BSP-2f-Src

