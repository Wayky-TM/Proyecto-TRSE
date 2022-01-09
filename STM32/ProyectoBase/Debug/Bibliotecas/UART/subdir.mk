################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/util.c 

OBJS += \
./Bibliotecas/UART/util.o 

C_DEPS += \
./Bibliotecas/UART/util.d 


# Each subdirectory must supply rules for building sources it contributes
Bibliotecas/UART/util.o: D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/util.c Bibliotecas/UART/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/Slave" -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bibliotecas-2f-UART

clean-Bibliotecas-2f-UART:
	-$(RM) ./Bibliotecas/UART/util.d ./Bibliotecas/UART/util.o

.PHONY: clean-Bibliotecas-2f-UART

