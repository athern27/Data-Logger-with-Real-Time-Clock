################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../OLED/fonts.c \
../OLED/ssd1306.c 

OBJS += \
./OLED/fonts.o \
./OLED/ssd1306.o 

C_DEPS += \
./OLED/fonts.d \
./OLED/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
OLED/%.o OLED/%.su OLED/%.cyclo: ../OLED/%.c OLED/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-OLED

clean-OLED:
	-$(RM) ./OLED/fonts.cyclo ./OLED/fonts.d ./OLED/fonts.o ./OLED/fonts.su ./OLED/ssd1306.cyclo ./OLED/ssd1306.d ./OLED/ssd1306.o ./OLED/ssd1306.su

.PHONY: clean-OLED

