################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/BasicMathFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/BayesFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/CommonTables" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/ComplexMathFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/ControllerFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/DistanceFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/FastMathFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/FilteringFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/InterpolationFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/MatrixFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/QuaternionMathFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/StatisticsFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/SupportFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/SVMFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/TransformFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

