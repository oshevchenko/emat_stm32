################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/event_queue.c \
../Src/main.c \
../Src/stm32_microrl_misc.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/timer.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/event_queue.o \
./Src/main.o \
./Src/stm32_microrl_misc.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/timer.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/event_queue.d \
./Src/main.d \
./Src/stm32_microrl_misc.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/timer.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xB -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Inc" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/microrl-master" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/oleksandr/workspace/stm32/emast_stm32_smart/emat_stm32_smart/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


