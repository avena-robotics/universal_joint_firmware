################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/main.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_api.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_config.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_interface.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_math.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_parameters.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_tasks.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/motor_control_protocol.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/motorcontrol.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/regular_conversion_manager.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/stm32g4xx_hal_msp.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/stm32g4xx_it.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/stm32g4xx_mc_it.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/ui_task.c \
C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/user_interface.c 

OBJS += \
./Application/User/main.o \
./Application/User/mc_api.o \
./Application/User/mc_config.o \
./Application/User/mc_interface.o \
./Application/User/mc_math.o \
./Application/User/mc_parameters.o \
./Application/User/mc_tasks.o \
./Application/User/motor_control_protocol.o \
./Application/User/motorcontrol.o \
./Application/User/regular_conversion_manager.o \
./Application/User/stm32g4xx_hal_msp.o \
./Application/User/stm32g4xx_it.o \
./Application/User/stm32g4xx_mc_it.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o \
./Application/User/ui_task.o \
./Application/User/user_interface.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/mc_api.d \
./Application/User/mc_config.d \
./Application/User/mc_interface.d \
./Application/User/mc_math.d \
./Application/User/mc_parameters.d \
./Application/User/mc_tasks.d \
./Application/User/motor_control_protocol.d \
./Application/User/motorcontrol.d \
./Application/User/regular_conversion_manager.d \
./Application/User/stm32g4xx_hal_msp.d \
./Application/User/stm32g4xx_it.d \
./Application/User/stm32g4xx_mc_it.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d \
./Application/User/ui_task.d \
./Application/User/user_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_api.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_api.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_config.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_config.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_interface.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_math.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_math.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_parameters.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_parameters.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_tasks.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/mc_tasks.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/motor_control_protocol.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/motor_control_protocol.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/motorcontrol.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/motorcontrol.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/regular_conversion_manager.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/regular_conversion_manager.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_hal_msp.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/stm32g4xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_it.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/stm32g4xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_mc_it.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/stm32g4xx_mc_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/ui_task.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/ui_task.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/user_interface.o: C:/Users/lukas/Documents/universal_joint_firmware/universal_joint_2.2_MC547_RI80/Src/user_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.7/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/main.d ./Application/User/main.o ./Application/User/mc_api.d ./Application/User/mc_api.o ./Application/User/mc_config.d ./Application/User/mc_config.o ./Application/User/mc_interface.d ./Application/User/mc_interface.o ./Application/User/mc_math.d ./Application/User/mc_math.o ./Application/User/mc_parameters.d ./Application/User/mc_parameters.o ./Application/User/mc_tasks.d ./Application/User/mc_tasks.o ./Application/User/motor_control_protocol.d ./Application/User/motor_control_protocol.o ./Application/User/motorcontrol.d ./Application/User/motorcontrol.o ./Application/User/regular_conversion_manager.d ./Application/User/regular_conversion_manager.o ./Application/User/stm32g4xx_hal_msp.d ./Application/User/stm32g4xx_hal_msp.o ./Application/User/stm32g4xx_it.d ./Application/User/stm32g4xx_it.o ./Application/User/stm32g4xx_mc_it.d ./Application/User/stm32g4xx_mc_it.o ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/sysmem.d ./Application/User/sysmem.o ./Application/User/ui_task.d ./Application/User/ui_task.o ./Application/User/user_interface.d ./Application/User/user_interface.o

.PHONY: clean-Application-2f-User

