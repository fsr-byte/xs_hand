################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/MPU6050_Motion_Driver/mllite/data_builder.c \
../Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.c \
../Middlewares/MPU6050_Motion_Driver/mllite/message_layer.c \
../Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.c \
../Middlewares/MPU6050_Motion_Driver/mllite/mlmath.c \
../Middlewares/MPU6050_Motion_Driver/mllite/mpl.c \
../Middlewares/MPU6050_Motion_Driver/mllite/results_holder.c \
../Middlewares/MPU6050_Motion_Driver/mllite/start_manager.c \
../Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.c 

OBJS += \
./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.o \
./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.o \
./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.o \
./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.o \
./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.o \
./Middlewares/MPU6050_Motion_Driver/mllite/mpl.o \
./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.o \
./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.o \
./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.o 

C_DEPS += \
./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.d \
./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.d \
./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.d \
./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.d \
./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.d \
./Middlewares/MPU6050_Motion_Driver/mllite/mpl.d \
./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.d \
./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.d \
./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/MPU6050_Motion_Driver/mllite/%.o Middlewares/MPU6050_Motion_Driver/mllite/%.su Middlewares/MPU6050_Motion_Driver/mllite/%.cyclo: ../Middlewares/MPU6050_Motion_Driver/mllite/%.c Middlewares/MPU6050_Motion_Driver/mllite/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL_TARGET_STM32F1 -DEMPL -DMPL_LOG_NDEBUG=0 -DMPU6050 -DUSE_DMP -DREMOVE_LOGGING -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fsr/code/xs_-hand/XS_HAND/APP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/BSP/Inc" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/eMPL" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/include" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/driver/stm32L" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/eMPL-hal" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/mllite" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/mpl" -I"C:/Users/fsr/code/xs_-hand/XS_HAND/Middlewares/MPU6050_Motion_Driver/porting" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-MPU6050_Motion_Driver-2f-mllite

clean-Middlewares-2f-MPU6050_Motion_Driver-2f-mllite:
	-$(RM) ./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.d ./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.o ./Middlewares/MPU6050_Motion_Driver/mllite/data_builder.su ./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.d ./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.o ./Middlewares/MPU6050_Motion_Driver/mllite/hal_outputs.su ./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.d ./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.o ./Middlewares/MPU6050_Motion_Driver/mllite/message_layer.su ./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.d ./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.o ./Middlewares/MPU6050_Motion_Driver/mllite/ml_math_func.su ./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.d ./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.o ./Middlewares/MPU6050_Motion_Driver/mllite/mlmath.su ./Middlewares/MPU6050_Motion_Driver/mllite/mpl.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/mpl.d ./Middlewares/MPU6050_Motion_Driver/mllite/mpl.o ./Middlewares/MPU6050_Motion_Driver/mllite/mpl.su ./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.d ./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.o ./Middlewares/MPU6050_Motion_Driver/mllite/results_holder.su ./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.d ./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.o ./Middlewares/MPU6050_Motion_Driver/mllite/start_manager.su ./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.cyclo ./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.d ./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.o ./Middlewares/MPU6050_Motion_Driver/mllite/storage_manager.su

.PHONY: clean-Middlewares-2f-MPU6050_Motion_Driver-2f-mllite

