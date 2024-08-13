################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/LwIP/src/apps/httpd/fs.c \
../Middlewares/Third_Party/LwIP/src/apps/httpd/httpd.c 

OBJS += \
./Middlewares/Third_Party/LwIP/src/apps/httpd/fs.o \
./Middlewares/Third_Party/LwIP/src/apps/httpd/httpd.o 

C_DEPS += \
./Middlewares/Third_Party/LwIP/src/apps/httpd/fs.d \
./Middlewares/Third_Party/LwIP/src/apps/httpd/httpd.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/LwIP/src/apps/httpd/%.o Middlewares/Third_Party/LwIP/src/apps/httpd/%.su Middlewares/Third_Party/LwIP/src/apps/httpd/%.cyclo: ../Middlewares/Third_Party/LwIP/src/apps/httpd/%.c Middlewares/Third_Party/LwIP/src/apps/httpd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F107xC -c -I../LWIP/App -I../LWIP/Target -I../Core/Inc -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/apps/httpd -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/posix -I../Middlewares/Third_Party/LwIP/src/include/posix/sys -I../Middlewares/Third_Party/LwIP/system/arch -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-LwIP-2f-src-2f-apps-2f-httpd

clean-Middlewares-2f-Third_Party-2f-LwIP-2f-src-2f-apps-2f-httpd:
	-$(RM) ./Middlewares/Third_Party/LwIP/src/apps/httpd/fs.cyclo ./Middlewares/Third_Party/LwIP/src/apps/httpd/fs.d ./Middlewares/Third_Party/LwIP/src/apps/httpd/fs.o ./Middlewares/Third_Party/LwIP/src/apps/httpd/fs.su ./Middlewares/Third_Party/LwIP/src/apps/httpd/httpd.cyclo ./Middlewares/Third_Party/LwIP/src/apps/httpd/httpd.d ./Middlewares/Third_Party/LwIP/src/apps/httpd/httpd.o ./Middlewares/Third_Party/LwIP/src/apps/httpd/httpd.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-LwIP-2f-src-2f-apps-2f-httpd

