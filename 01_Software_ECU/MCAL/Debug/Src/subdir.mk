################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/mcalADC.c \
../Src/mcalDMAC.c \
../Src/mcalEXTI.c \
../Src/mcalFlash.c \
../Src/mcalGPIO.c \
../Src/mcalI2C.c \
../Src/mcalRCC.c \
../Src/mcalSPI.c \
../Src/mcalSysTick.c \
../Src/mcalUsart.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/mcalADC.o \
./Src/mcalDMAC.o \
./Src/mcalEXTI.o \
./Src/mcalFlash.o \
./Src/mcalGPIO.o \
./Src/mcalI2C.o \
./Src/mcalRCC.o \
./Src/mcalSPI.o \
./Src/mcalSysTick.o \
./Src/mcalUsart.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/mcalADC.d \
./Src/mcalDMAC.d \
./Src/mcalEXTI.d \
./Src/mcalFlash.d \
./Src/mcalGPIO.d \
./Src/mcalI2C.d \
./Src/mcalRCC.d \
./Src/mcalSPI.d \
./Src/mcalSysTick.d \
./Src/mcalUsart.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I"D:/Projekte/tellur/01_Software_ECU/MCAL/Inc" -I"D:/Projekte/tellur/01_Software_ECU/MCAL/Src" -I"D:/Projekte/tellur/01_Software_ECU/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Projekte/tellur/01_Software_ECU/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/mcalADC.d ./Src/mcalADC.o ./Src/mcalADC.su ./Src/mcalDMAC.d ./Src/mcalDMAC.o ./Src/mcalDMAC.su ./Src/mcalEXTI.d ./Src/mcalEXTI.o ./Src/mcalEXTI.su ./Src/mcalFlash.d ./Src/mcalFlash.o ./Src/mcalFlash.su ./Src/mcalGPIO.d ./Src/mcalGPIO.o ./Src/mcalGPIO.su ./Src/mcalI2C.d ./Src/mcalI2C.o ./Src/mcalI2C.su ./Src/mcalRCC.d ./Src/mcalRCC.o ./Src/mcalRCC.su ./Src/mcalSPI.d ./Src/mcalSPI.o ./Src/mcalSPI.su ./Src/mcalSysTick.d ./Src/mcalSysTick.o ./Src/mcalSysTick.su ./Src/mcalUsart.d ./Src/mcalUsart.o ./Src/mcalUsart.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

