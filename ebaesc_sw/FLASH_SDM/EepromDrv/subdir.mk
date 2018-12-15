################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../EepromDrv/EepromDrv.c" \

C_SRCS += \
../EepromDrv/EepromDrv.c \

OBJS += \
./EepromDrv/EepromDrv_c.obj \

OBJS_QUOTED += \
"./EepromDrv/EepromDrv_c.obj" \

C_DEPS += \
./EepromDrv/EepromDrv_c.d \

C_DEPS_QUOTED += \
"./EepromDrv/EepromDrv_c.d" \

OBJS_OS_FORMAT += \
./EepromDrv/EepromDrv_c.obj \


# Each subdirectory must supply rules for building sources it contributes
EepromDrv/EepromDrv_c.obj: ../EepromDrv/EepromDrv.c
	@echo 'Building file: $<'
	@echo 'Executing target #53 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"EepromDrv/EepromDrv.args" -o "EepromDrv/EepromDrv_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

EepromDrv/%.d: ../EepromDrv/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '


