################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/buzzer.c \
../Core/Src/get_ir_reading.c \
../Core/Src/main.c \
../Core/Src/motor_control.c \
../Core/Src/mpu6050.c \
../Core/Src/pid_position.c \
../Core/Src/push_btn_library.c \
../Core/Src/rotate_angle.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/variables.c 

OBJS += \
./Core/Src/buzzer.o \
./Core/Src/get_ir_reading.o \
./Core/Src/main.o \
./Core/Src/motor_control.o \
./Core/Src/mpu6050.o \
./Core/Src/pid_position.o \
./Core/Src/push_btn_library.o \
./Core/Src/rotate_angle.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/variables.o 

C_DEPS += \
./Core/Src/buzzer.d \
./Core/Src/get_ir_reading.d \
./Core/Src/main.d \
./Core/Src/motor_control.d \
./Core/Src/mpu6050.d \
./Core/Src/pid_position.d \
./Core/Src/push_btn_library.d \
./Core/Src/rotate_angle.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/variables.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/buzzer.cyclo ./Core/Src/buzzer.d ./Core/Src/buzzer.o ./Core/Src/buzzer.su ./Core/Src/get_ir_reading.cyclo ./Core/Src/get_ir_reading.d ./Core/Src/get_ir_reading.o ./Core/Src/get_ir_reading.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor_control.cyclo ./Core/Src/motor_control.d ./Core/Src/motor_control.o ./Core/Src/motor_control.su ./Core/Src/mpu6050.cyclo ./Core/Src/mpu6050.d ./Core/Src/mpu6050.o ./Core/Src/mpu6050.su ./Core/Src/pid_position.cyclo ./Core/Src/pid_position.d ./Core/Src/pid_position.o ./Core/Src/pid_position.su ./Core/Src/push_btn_library.cyclo ./Core/Src/push_btn_library.d ./Core/Src/push_btn_library.o ./Core/Src/push_btn_library.su ./Core/Src/rotate_angle.cyclo ./Core/Src/rotate_angle.d ./Core/Src/rotate_angle.o ./Core/Src/rotate_angle.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/variables.cyclo ./Core/Src/variables.d ./Core/Src/variables.o ./Core/Src/variables.su

.PHONY: clean-Core-2f-Src

