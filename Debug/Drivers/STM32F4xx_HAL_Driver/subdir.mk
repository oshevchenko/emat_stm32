################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_cortex.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma_ex.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ex.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ramfunc.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_gpio.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pcd.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pcd_ex.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr_ex.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc_ex.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim_ex.c \
../Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usb.c 

OBJS += \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_cortex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ramfunc.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_gpio.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pcd.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pcd_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usb.o 

C_DEPS += \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_cortex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ramfunc.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_gpio.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pcd.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pcd_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usb.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal.o: /home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F407xx -I"/home/oleksandr/workspace/stm32/emat_stm32/Inc" -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/STM32F4xx_HAL_Driver/Inc -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/CMSIS/Include -I"/home/oleksandr/workspace/stm32/emat_stm32/microrl-master"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/%.o: ../Drivers/STM32F4xx_HAL_Driver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F407xx -I"/home/oleksandr/workspace/stm32/emat_stm32/Inc" -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/STM32F4xx_HAL_Driver/Inc -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/home/oleksandr/STM32Cube/Repository/STM32Cube_FW_F4_V1.21.0/Drivers/CMSIS/Include -I"/home/oleksandr/workspace/stm32/emat_stm32/microrl-master"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


