################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/mcalTimer/advancedControl.c \
../Src/mcalTimer/commonCapComp.c \
../Src/mcalTimer/deprecated.c \
../Src/mcalTimer/dma.c \
../Src/mcalTimer/events.c \
../Src/mcalTimer/inputCapture.c \
../Src/mcalTimer/interrupt.c \
../Src/mcalTimer/masterSlave.c \
../Src/mcalTimer/mcalTimer.c \
../Src/mcalTimer/outputCompare.c \
../Src/mcalTimer/verification.c 

OBJS += \
./Src/mcalTimer/advancedControl.o \
./Src/mcalTimer/commonCapComp.o \
./Src/mcalTimer/deprecated.o \
./Src/mcalTimer/dma.o \
./Src/mcalTimer/events.o \
./Src/mcalTimer/inputCapture.o \
./Src/mcalTimer/interrupt.o \
./Src/mcalTimer/masterSlave.o \
./Src/mcalTimer/mcalTimer.o \
./Src/mcalTimer/outputCompare.o \
./Src/mcalTimer/verification.o 

C_DEPS += \
./Src/mcalTimer/advancedControl.d \
./Src/mcalTimer/commonCapComp.d \
./Src/mcalTimer/deprecated.d \
./Src/mcalTimer/dma.d \
./Src/mcalTimer/events.d \
./Src/mcalTimer/inputCapture.d \
./Src/mcalTimer/interrupt.d \
./Src/mcalTimer/masterSlave.d \
./Src/mcalTimer/mcalTimer.d \
./Src/mcalTimer/outputCompare.d \
./Src/mcalTimer/verification.d 


# Each subdirectory must supply rules for building sources it contributes
Src/mcalTimer/%.o Src/mcalTimer/%.su: ../Src/mcalTimer/%.c Src/mcalTimer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I"D:/Projekte/tellur/01_Software_ECU/MCAL/Inc" -I"D:/Projekte/tellur/01_Software_ECU/MCAL/Src" -I"D:/Projekte/tellur/01_Software_ECU/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Projekte/tellur/01_Software_ECU/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-mcalTimer

clean-Src-2f-mcalTimer:
	-$(RM) ./Src/mcalTimer/advancedControl.d ./Src/mcalTimer/advancedControl.o ./Src/mcalTimer/advancedControl.su ./Src/mcalTimer/commonCapComp.d ./Src/mcalTimer/commonCapComp.o ./Src/mcalTimer/commonCapComp.su ./Src/mcalTimer/deprecated.d ./Src/mcalTimer/deprecated.o ./Src/mcalTimer/deprecated.su ./Src/mcalTimer/dma.d ./Src/mcalTimer/dma.o ./Src/mcalTimer/dma.su ./Src/mcalTimer/events.d ./Src/mcalTimer/events.o ./Src/mcalTimer/events.su ./Src/mcalTimer/inputCapture.d ./Src/mcalTimer/inputCapture.o ./Src/mcalTimer/inputCapture.su ./Src/mcalTimer/interrupt.d ./Src/mcalTimer/interrupt.o ./Src/mcalTimer/interrupt.su ./Src/mcalTimer/masterSlave.d ./Src/mcalTimer/masterSlave.o ./Src/mcalTimer/masterSlave.su ./Src/mcalTimer/mcalTimer.d ./Src/mcalTimer/mcalTimer.o ./Src/mcalTimer/mcalTimer.su ./Src/mcalTimer/outputCompare.d ./Src/mcalTimer/outputCompare.o ./Src/mcalTimer/outputCompare.su ./Src/mcalTimer/verification.d ./Src/mcalTimer/verification.o ./Src/mcalTimer/verification.su

.PHONY: clean-Src-2f-mcalTimer

