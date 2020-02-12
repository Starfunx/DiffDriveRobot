################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l476rgtx.s 

OBJS += \
./Core/Startup/startup_stm32l476rgtx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s
	arm-none-eabi-gcc -u_printf_float -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

