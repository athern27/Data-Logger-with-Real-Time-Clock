################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Internal_Temp/Internal_Temp.c 

OBJS += \
./Internal_Temp/Internal_Temp.o 

C_DEPS += \
./Internal_Temp/Internal_Temp.d 


# Each subdirectory must supply rules for building sources it contributes
Internal_Temp/%.o Internal_Temp/%.su Internal_Temp/%.cyclo: ../Internal_Temp/%.c Internal_Temp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS_V2" -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/Middlewares/Third_Party/FreeRTOS/org/Source/include" -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/Middlewares/Third_Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/DMA_ADC" -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/SD_card" -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/UART" -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/OLED" -I../Core/Inc -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/SEGGER/Config" -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/SEGGER/OS" -I"C:/Users/karti/STM32CubeIDE/workspace_1.13.1/RTOS_project_segger/SEGGER/SEGGER" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Internal_Temp

clean-Internal_Temp:
	-$(RM) ./Internal_Temp/Internal_Temp.cyclo ./Internal_Temp/Internal_Temp.d ./Internal_Temp/Internal_Temp.o ./Internal_Temp/Internal_Temp.su

.PHONY: clean-Internal_Temp
