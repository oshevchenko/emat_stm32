################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../microrl-master/src/microrl.c 

OBJS += \
./microrl-master/src/microrl.o 

C_DEPS += \
./microrl-master/src/microrl.d 


# Each subdirectory must supply rules for building sources it contributes
microrl-master/src/%.o: ../microrl-master/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xB -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Inc" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/microrl-master" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


