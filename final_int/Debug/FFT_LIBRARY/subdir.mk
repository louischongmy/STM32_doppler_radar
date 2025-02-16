################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FFT_LIBRARY/FFT.c 

OBJS += \
./FFT_LIBRARY/FFT.o 

C_DEPS += \
./FFT_LIBRARY/FFT.d 


# Each subdirectory must supply rules for building sources it contributes
FFT_LIBRARY/%.o FFT_LIBRARY/%.su FFT_LIBRARY/%.cyclo: ../FFT_LIBRARY/%.c FFT_LIBRARY/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/CMSIS/DSP_LIBRARY/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/louis/Downloads/final_int-20241226T120753Z-001/final_int/FFT_LIBRARY" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FFT_LIBRARY

clean-FFT_LIBRARY:
	-$(RM) ./FFT_LIBRARY/FFT.cyclo ./FFT_LIBRARY/FFT.d ./FFT_LIBRARY/FFT.o ./FFT_LIBRARY/FFT.su

.PHONY: clean-FFT_LIBRARY

