################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Controller.c \
../Core/Src/Cytron_MDXX.c \
../Core/Src/PWM.c \
../Core/Src/app_freertos.c \
../Core/Src/config.c \
../Core/Src/custom_memory_manager.c \
../Core/Src/dma.c \
../Core/Src/dma_transport.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/microros_allocators.c \
../Core/Src/microros_time.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_hal_timebase_tim.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/Controller.o \
./Core/Src/Cytron_MDXX.o \
./Core/Src/PWM.o \
./Core/Src/app_freertos.o \
./Core/Src/config.o \
./Core/Src/custom_memory_manager.o \
./Core/Src/dma.o \
./Core/Src/dma_transport.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/microros_allocators.o \
./Core/Src/microros_time.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_hal_timebase_tim.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/Controller.d \
./Core/Src/Cytron_MDXX.d \
./Core/Src/PWM.d \
./Core/Src/app_freertos.d \
./Core/Src/config.d \
./Core/Src/custom_memory_manager.d \
./Core/Src/dma.d \
./Core/Src/dma_transport.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/microros_allocators.d \
./Core/Src/microros_time.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_hal_timebase_tim.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/BasicMathFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/BayesFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/CommonTables" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/ComplexMathFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/ControllerFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/DistanceFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/FastMathFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/FilteringFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/InterpolationFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/MatrixFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/QuaternionMathFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/StatisticsFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/SupportFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/SVMFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/TransformFunctions" -I"/home/b/cleaner_ws/firmware/cleaner_uros/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Controller.cyclo ./Core/Src/Controller.d ./Core/Src/Controller.o ./Core/Src/Controller.su ./Core/Src/Cytron_MDXX.cyclo ./Core/Src/Cytron_MDXX.d ./Core/Src/Cytron_MDXX.o ./Core/Src/Cytron_MDXX.su ./Core/Src/PWM.cyclo ./Core/Src/PWM.d ./Core/Src/PWM.o ./Core/Src/PWM.su ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/config.cyclo ./Core/Src/config.d ./Core/Src/config.o ./Core/Src/config.su ./Core/Src/custom_memory_manager.cyclo ./Core/Src/custom_memory_manager.d ./Core/Src/custom_memory_manager.o ./Core/Src/custom_memory_manager.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/dma_transport.cyclo ./Core/Src/dma_transport.d ./Core/Src/dma_transport.o ./Core/Src/dma_transport.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/microros_allocators.cyclo ./Core/Src/microros_allocators.d ./Core/Src/microros_allocators.o ./Core/Src/microros_allocators.su ./Core/Src/microros_time.cyclo ./Core/Src/microros_time.d ./Core/Src/microros_time.o ./Core/Src/microros_time.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_hal_timebase_tim.cyclo ./Core/Src/stm32g4xx_hal_timebase_tim.d ./Core/Src/stm32g4xx_hal_timebase_tim.o ./Core/Src/stm32g4xx_hal_timebase_tim.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

