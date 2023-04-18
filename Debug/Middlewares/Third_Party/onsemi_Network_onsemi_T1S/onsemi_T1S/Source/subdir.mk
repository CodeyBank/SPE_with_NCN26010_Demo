################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/NCN26010.c \
../Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_OS.c \
../Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_TCP-IP.c \
../Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_STM32_Hardware.c 

OBJS += \
./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/NCN26010.o \
./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_OS.o \
./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_TCP-IP.o \
./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_STM32_Hardware.o 

C_DEPS += \
./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/NCN26010.d \
./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_OS.d \
./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_TCP-IP.d \
./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_STM32_Hardware.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/%.o Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/%.su: ../Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/%.c Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/FreeRTOS-Plus-TCP/include -I../Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-onsemi_Network_onsemi_T1S-2f-onsemi_T1S-2f-Source

clean-Middlewares-2f-Third_Party-2f-onsemi_Network_onsemi_T1S-2f-onsemi_T1S-2f-Source:
	-$(RM) ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/NCN26010.d ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/NCN26010.o ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/NCN26010.su ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_OS.d ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_OS.o ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_OS.su ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_TCP-IP.d ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_TCP-IP.o ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_FreeRTOS_TCP-IP.su ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_STM32_Hardware.d ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_STM32_Hardware.o ./Middlewares/Third_Party/onsemi_Network_onsemi_T1S/onsemi_T1S/Source/T1S_STM32_Hardware.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-onsemi_Network_onsemi_T1S-2f-onsemi_T1S-2f-Source

