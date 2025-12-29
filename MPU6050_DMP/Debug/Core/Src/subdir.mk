################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/IOI2C.c \
../Core/Src/KF.c \
../Core/Src/app_control.c \
../Core/Src/delay.c \
../Core/Src/encoder.c \
../Core/Src/filtrer.c \
../Core/Src/inv_mpu.c \
../Core/Src/key.c \
../Core/Src/led.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/mpu6050.c \
../Core/Src/pid_control.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/IOI2C.o \
./Core/Src/KF.o \
./Core/Src/app_control.o \
./Core/Src/delay.o \
./Core/Src/encoder.o \
./Core/Src/filtrer.o \
./Core/Src/inv_mpu.o \
./Core/Src/key.o \
./Core/Src/led.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/mpu6050.o \
./Core/Src/pid_control.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/IOI2C.d \
./Core/Src/KF.d \
./Core/Src/app_control.d \
./Core/Src/delay.d \
./Core/Src/encoder.d \
./Core/Src/filtrer.d \
./Core/Src/inv_mpu.d \
./Core/Src/key.d \
./Core/Src/led.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/mpu6050.d \
./Core/Src/pid_control.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -fshort-enums -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/IOI2C.cyclo ./Core/Src/IOI2C.d ./Core/Src/IOI2C.o ./Core/Src/IOI2C.su ./Core/Src/KF.cyclo ./Core/Src/KF.d ./Core/Src/KF.o ./Core/Src/KF.su ./Core/Src/app_control.cyclo ./Core/Src/app_control.d ./Core/Src/app_control.o ./Core/Src/app_control.su ./Core/Src/delay.cyclo ./Core/Src/delay.d ./Core/Src/delay.o ./Core/Src/delay.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/filtrer.cyclo ./Core/Src/filtrer.d ./Core/Src/filtrer.o ./Core/Src/filtrer.su ./Core/Src/inv_mpu.cyclo ./Core/Src/inv_mpu.d ./Core/Src/inv_mpu.o ./Core/Src/inv_mpu.su ./Core/Src/key.cyclo ./Core/Src/key.d ./Core/Src/key.o ./Core/Src/key.su ./Core/Src/led.cyclo ./Core/Src/led.d ./Core/Src/led.o ./Core/Src/led.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/mpu6050.cyclo ./Core/Src/mpu6050.d ./Core/Src/mpu6050.o ./Core/Src/mpu6050.su ./Core/Src/pid_control.cyclo ./Core/Src/pid_control.d ./Core/Src/pid_control.o ./Core/Src/pid_control.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src

