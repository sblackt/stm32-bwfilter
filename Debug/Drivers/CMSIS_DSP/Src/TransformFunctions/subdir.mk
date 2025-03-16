################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal2.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal_f16.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_f32.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f16.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f32.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f64.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q15.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q31.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_radix8_f32.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_f32.c \
../Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_init_f32.c 

OBJS += \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal2.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal_f16.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_f32.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f16.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f32.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f64.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q15.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q31.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_radix8_f32.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_f32.o \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_init_f32.o 

C_DEPS += \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal2.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal_f16.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_f32.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f16.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f32.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f64.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q15.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q31.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_radix8_f32.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_f32.d \
./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_init_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Src/TransformFunctions/%.o Drivers/CMSIS_DSP/Src/TransformFunctions/%.su Drivers/CMSIS_DSP/Src/TransformFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Src/TransformFunctions/%.c Drivers/CMSIS_DSP/Src/TransformFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F4 -DARM_MATH_4 -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Drivers/CMSIS_DSP/Inc/dsp -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS_DSP/Inc -I../Drivers/CMSIS_DSP/PrivateInclude -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Src-2f-TransformFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Src-2f-TransformFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal2.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal2.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal2.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal2.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal_f16.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal_f16.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal_f16.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_bitreversal_f16.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_f32.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_f32.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_f32.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_f32.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f16.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f16.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f16.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f16.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f32.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f32.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f32.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f32.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f64.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f64.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f64.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_f64.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q15.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q15.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q15.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q15.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q31.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q31.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q31.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_init_q31.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_radix8_f32.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_radix8_f32.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_radix8_f32.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_cfft_radix8_f32.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_f32.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_f32.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_f32.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_f32.su ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_init_f32.cyclo ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_init_f32.d ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_init_f32.o ./Drivers/CMSIS_DSP/Src/TransformFunctions/arm_rfft_fast_init_f32.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Src-2f-TransformFunctions

