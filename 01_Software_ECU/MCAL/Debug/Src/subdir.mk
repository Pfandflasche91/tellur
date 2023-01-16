################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MCAL.c \
../Src/mcalDMAC.c \
../Src/mcalEXTI.c \
../Src/mcalGPIO.c \
../Src/mcalI2C.c \
../Src/mcalSPI.c \
../Src/mcalSysTick.c \
../Src/mcalSystem.c \
../Src/mcalTimer.c \
../Src/mcalUsart.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/MCAL.o \
./Src/mcalDMAC.o \
./Src/mcalEXTI.o \
./Src/mcalGPIO.o \
./Src/mcalI2C.o \
./Src/mcalSPI.o \
./Src/mcalSysTick.o \
./Src/mcalSystem.o \
./Src/mcalTimer.o \
./Src/mcalUsart.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/MCAL.d \
./Src/mcalDMAC.d \
./Src/mcalEXTI.d \
./Src/mcalGPIO.d \
./Src/mcalI2C.d \
./Src/mcalSPI.d \
./Src/mcalSysTick.d \
./Src/mcalSystem.d \
./Src/mcalTimer.d \
./Src/mcalUsart.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I"D:/Projekte/tellur/01_Software_ECU/MCAL/Inc" -I"D:/Projekte/tellur/01_Software_ECU/MCAL/Src" -I"D:/Projekte/tellur/01_Software_ECU/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Projekte/tellur/01_Software_ECU/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/MCAL.d ./Src/MCAL.o ./Src/MCAL.su ./Src/mcalDMAC.d ./Src/mcalDMAC.o ./Src/mcalDMAC.su ./Src/mcalEXTI.d ./Src/mcalEXTI.o ./Src/mcalEXTI.su ./Src/mcalGPIO.d ./Src/mcalGPIO.o ./Src/mcalGPIO.su ./Src/mcalI2C.d ./Src/mcalI2C.o ./Src/mcalI2C.su ./Src/mcalSPI.d ./Src/mcalSPI.o ./Src/mcalSPI.su ./Src/mcalSysTick.d ./Src/mcalSysTick.o ./Src/mcalSysTick.su ./Src/mcalSystem.d ./Src/mcalSystem.o ./Src/mcalSystem.su ./Src/mcalTimer.d ./Src/mcalTimer.o ./Src/mcalTimer.su ./Src/mcalUsart.d ./Src/mcalUsart.o ./Src/mcalUsart.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

