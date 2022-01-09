################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/Slave/UART.c \
D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/Slave/server.c 

OBJS += \
./Bibliotecas/UART/Slave/UART.o \
./Bibliotecas/UART/Slave/server.o 

C_DEPS += \
./Bibliotecas/UART/Slave/UART.d \
./Bibliotecas/UART/Slave/server.d 


# Each subdirectory must supply rules for building sources it contributes
Bibliotecas/UART/Slave/UART.o: D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/Slave/UART.c Bibliotecas/UART/Slave/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/Slave" -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Bibliotecas/UART/Slave/server.o: D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/Slave/server.c Bibliotecas/UART/Slave/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART/Slave" -I"D:/Dropbox/Importantes/UNI/MISE/TRSE/Proyecto/Repo/Proyecto-TRSE/Bibliotecas/UART" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bibliotecas-2f-UART-2f-Slave

clean-Bibliotecas-2f-UART-2f-Slave:
	-$(RM) ./Bibliotecas/UART/Slave/UART.d ./Bibliotecas/UART/Slave/UART.o ./Bibliotecas/UART/Slave/server.d ./Bibliotecas/UART/Slave/server.o

.PHONY: clean-Bibliotecas-2f-UART-2f-Slave

