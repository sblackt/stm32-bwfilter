################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Src/arm_common_tables.c \
../Drivers/CMSIS_DSP/Src/arm_const_structs.c 

OBJS += \
./Drivers/CMSIS_DSP/Src/arm_common_tables.o \
./Drivers/CMSIS_DSP/Src/arm_const_structs.o 

C_DEPS += \
./Drivers/CMSIS_DSP/Src/arm_common_tables.d \
./Drivers/CMSIS_DSP/Src/arm_const_structs.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Src/%.o Drivers/CMSIS_DSP/Src/%.su Drivers/CMSIS_DSP/Src/%.cyclo: ../Drivers/CMSIS_DSP/Src/%.c Drivers/CMSIS_DSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F4 -DARM_MATH_4 -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Drivers/CMSIS_DSP/Inc/dsp -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS_DSP/Inc -I../Drivers/CMSIS_DSP/PrivateInclude -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Src

clean-Drivers-2f-CMSIS_DSP-2f-Src:
	-$(RM) ./Drivers/CMSIS_DSP/Src/arm_common_tables.cyclo ./Drivers/CMSIS_DSP/Src/arm_common_tables.d ./Drivers/CMSIS_DSP/Src/arm_common_tables.o ./Drivers/CMSIS_DSP/Src/arm_common_tables.su ./Drivers/CMSIS_DSP/Src/arm_const_structs.cyclo ./Drivers/CMSIS_DSP/Src/arm_const_structs.d ./Drivers/CMSIS_DSP/Src/arm_const_structs.o ./Drivers/CMSIS_DSP/Src/arm_const_structs.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Src

