################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Fusion/FusionAhrs.c \
../Core/Fusion/FusionBias.c \
../Core/Fusion/FusionCompass.c 

OBJS += \
./Core/Fusion/FusionAhrs.o \
./Core/Fusion/FusionBias.o \
./Core/Fusion/FusionCompass.o 

C_DEPS += \
./Core/Fusion/FusionAhrs.d \
./Core/Fusion/FusionBias.d \
./Core/Fusion/FusionCompass.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Fusion/%.o Core/Fusion/%.su Core/Fusion/%.cyclo: ../Core/Fusion/%.c Core/Fusion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Fusion

clean-Core-2f-Fusion:
	-$(RM) ./Core/Fusion/FusionAhrs.cyclo ./Core/Fusion/FusionAhrs.d ./Core/Fusion/FusionAhrs.o ./Core/Fusion/FusionAhrs.su ./Core/Fusion/FusionBias.cyclo ./Core/Fusion/FusionBias.d ./Core/Fusion/FusionBias.o ./Core/Fusion/FusionBias.su ./Core/Fusion/FusionCompass.cyclo ./Core/Fusion/FusionCompass.d ./Core/Fusion/FusionCompass.o ./Core/Fusion/FusionCompass.su

.PHONY: clean-Core-2f-Fusion

