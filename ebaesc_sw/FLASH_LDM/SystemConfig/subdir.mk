################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../SystemConfig/appconfig.c" \
"../SystemConfig/startup.c" \
"../SystemConfig/vectors.c" \

C_SRCS += \
../SystemConfig/appconfig.c \
../SystemConfig/startup.c \
../SystemConfig/vectors.c \

OBJS += \
./SystemConfig/appconfig_c.obj \
./SystemConfig/startup_c.obj \
./SystemConfig/vectors_c.obj \

OBJS_QUOTED += \
"./SystemConfig/appconfig_c.obj" \
"./SystemConfig/startup_c.obj" \
"./SystemConfig/vectors_c.obj" \

C_DEPS += \
./SystemConfig/appconfig_c.d \
./SystemConfig/startup_c.d \
./SystemConfig/vectors_c.d \

C_DEPS_QUOTED += \
"./SystemConfig/appconfig_c.d" \
"./SystemConfig/startup_c.d" \
"./SystemConfig/vectors_c.d" \

OBJS_OS_FORMAT += \
./SystemConfig/appconfig_c.obj \
./SystemConfig/startup_c.obj \
./SystemConfig/vectors_c.obj \


# Each subdirectory must supply rules for building sources it contributes
SystemConfig/appconfig_c.obj: ../SystemConfig/appconfig.c
	@echo 'Building file: $<'
	@echo 'Executing target #48 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"SystemConfig/appconfig.args" -o "SystemConfig/appconfig_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

SystemConfig/%.d: ../SystemConfig/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

SystemConfig/startup_c.obj: ../SystemConfig/startup.c
	@echo 'Building file: $<'
	@echo 'Executing target #49 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"SystemConfig/startup.args" -o "SystemConfig/startup_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

SystemConfig/vectors_c.obj: ../SystemConfig/vectors.c
	@echo 'Building file: $<'
	@echo 'Executing target #50 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"SystemConfig/vectors.args" -o "SystemConfig/vectors_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '


