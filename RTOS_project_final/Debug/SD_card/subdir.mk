################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SD_card/fatfs_sd.c 

OBJS += \
./SD_card/fatfs_sd.o 

C_DEPS += \
./SD_card/fatfs_sd.d 


# Each subdirectory must supply rules for building sources it contributes
SD_card/%.o SD_card/%.su SD_card/%.cyclo: ../SD_card/%.c SD_card/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/karti/Desktop/AESD/RTOS_project_101124/SD_card" -I"C:/Users/karti/Desktop/AESD/RTOS_project_101124/OLED" -I"C:/Users/karti/Desktop/AESD/RTOS_project_101124/DMA_ADC" -I"C:/Users/karti/Desktop/AESD/RTOS_project_101124/UART" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SD_card

clean-SD_card:
	-$(RM) ./SD_card/fatfs_sd.cyclo ./SD_card/fatfs_sd.d ./SD_card/fatfs_sd.o ./SD_card/fatfs_sd.su

.PHONY: clean-SD_card

