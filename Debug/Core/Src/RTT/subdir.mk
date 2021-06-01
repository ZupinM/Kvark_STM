################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/RTT/SEGGER_RTT.c \
../Core/Src/RTT/SEGGER_RTT_Syscalls_GCC.c \
../Core/Src/RTT/SEGGER_RTT_Syscalls_IAR.c \
../Core/Src/RTT/SEGGER_RTT_Syscalls_KEIL.c \
../Core/Src/RTT/SEGGER_RTT_Syscalls_SES.c \
../Core/Src/RTT/SEGGER_RTT_printf.c 

S_UPPER_SRCS += \
../Core/Src/RTT/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./Core/Src/RTT/SEGGER_RTT.o \
./Core/Src/RTT/SEGGER_RTT_ASM_ARMv7M.o \
./Core/Src/RTT/SEGGER_RTT_Syscalls_GCC.o \
./Core/Src/RTT/SEGGER_RTT_Syscalls_IAR.o \
./Core/Src/RTT/SEGGER_RTT_Syscalls_KEIL.o \
./Core/Src/RTT/SEGGER_RTT_Syscalls_SES.o \
./Core/Src/RTT/SEGGER_RTT_printf.o 

S_UPPER_DEPS += \
./Core/Src/RTT/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./Core/Src/RTT/SEGGER_RTT.d \
./Core/Src/RTT/SEGGER_RTT_Syscalls_GCC.d \
./Core/Src/RTT/SEGGER_RTT_Syscalls_IAR.d \
./Core/Src/RTT/SEGGER_RTT_Syscalls_KEIL.d \
./Core/Src/RTT/SEGGER_RTT_Syscalls_SES.d \
./Core/Src/RTT/SEGGER_RTT_printf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/RTT/SEGGER_RTT.o: ../Core/Src/RTT/SEGGER_RTT.c Core/Src/RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/RTT/SEGGER_RTT.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/RTT/SEGGER_RTT_ASM_ARMv7M.o: ../Core/Src/RTT/SEGGER_RTT_ASM_ARMv7M.S Core/Src/RTT/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Src/RTT/SEGGER_RTT_ASM_ARMv7M.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Core/Src/RTT/SEGGER_RTT_Syscalls_GCC.o: ../Core/Src/RTT/SEGGER_RTT_Syscalls_GCC.c Core/Src/RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/RTT/SEGGER_RTT_Syscalls_GCC.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/RTT/SEGGER_RTT_Syscalls_IAR.o: ../Core/Src/RTT/SEGGER_RTT_Syscalls_IAR.c Core/Src/RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/RTT/SEGGER_RTT_Syscalls_IAR.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/RTT/SEGGER_RTT_Syscalls_KEIL.o: ../Core/Src/RTT/SEGGER_RTT_Syscalls_KEIL.c Core/Src/RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/RTT/SEGGER_RTT_Syscalls_KEIL.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/RTT/SEGGER_RTT_Syscalls_SES.o: ../Core/Src/RTT/SEGGER_RTT_Syscalls_SES.c Core/Src/RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/RTT/SEGGER_RTT_Syscalls_SES.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/RTT/SEGGER_RTT_printf.o: ../Core/Src/RTT/SEGGER_RTT_printf.c Core/Src/RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/RTT/SEGGER_RTT_printf.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

