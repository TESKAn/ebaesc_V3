################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../sources/RS485comm.c" \
"../sources/drv8301.c" \
"../sources/events.c" \
"../sources/functions.c" \
"../sources/init.c" \
"../sources/systemStates.c" \
"../sources/var.c" \

C_SRCS += \
../sources/RS485comm.c \
../sources/drv8301.c \
../sources/events.c \
../sources/functions.c \
../sources/init.c \
../sources/systemStates.c \
../sources/var.c \

OBJS += \
./sources/RS485comm_c.obj \
./sources/drv8301_c.obj \
./sources/events_c.obj \
./sources/functions_c.obj \
./sources/init_c.obj \
./sources/systemStates_c.obj \
./sources/var_c.obj \

OBJS_QUOTED += \
"./sources/RS485comm_c.obj" \
"./sources/drv8301_c.obj" \
"./sources/events_c.obj" \
"./sources/functions_c.obj" \
"./sources/init_c.obj" \
"./sources/systemStates_c.obj" \
"./sources/var_c.obj" \

C_DEPS += \
./sources/RS485comm_c.d \
./sources/drv8301_c.d \
./sources/events_c.d \
./sources/functions_c.d \
./sources/init_c.d \
./sources/systemStates_c.d \
./sources/var_c.d \

C_DEPS_QUOTED += \
"./sources/RS485comm_c.d" \
"./sources/drv8301_c.d" \
"./sources/events_c.d" \
"./sources/functions_c.d" \
"./sources/init_c.d" \
"./sources/systemStates_c.d" \
"./sources/var_c.d" \

OBJS_OS_FORMAT += \
./sources/RS485comm_c.obj \
./sources/drv8301_c.obj \
./sources/events_c.obj \
./sources/functions_c.obj \
./sources/init_c.obj \
./sources/systemStates_c.obj \
./sources/var_c.obj \


# Each subdirectory must supply rules for building sources it contributes
sources/RS485comm_c.obj: ../sources/RS485comm.c
	@echo 'Building file: $<'
	@echo 'Executing target #39 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"sources/RS485comm.args" -o "sources/RS485comm_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

sources/%.d: ../sources/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

sources/drv8301_c.obj: ../sources/drv8301.c
	@echo 'Building file: $<'
	@echo 'Executing target #40 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"sources/drv8301.args" -o "sources/drv8301_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

sources/events_c.obj: ../sources/events.c
	@echo 'Building file: $<'
	@echo 'Executing target #41 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"sources/events.args" -o "sources/events_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

sources/functions_c.obj: ../sources/functions.c
	@echo 'Building file: $<'
	@echo 'Executing target #42 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"sources/functions.args" -o "sources/functions_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

sources/init_c.obj: ../sources/init.c
	@echo 'Building file: $<'
	@echo 'Executing target #43 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"sources/init.args" -o "sources/init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

sources/systemStates_c.obj: ../sources/systemStates.c
	@echo 'Building file: $<'
	@echo 'Executing target #44 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"sources/systemStates.args" -o "sources/systemStates_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

sources/var_c.obj: ../sources/var.c
	@echo 'Building file: $<'
	@echo 'Executing target #45 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"sources/var.args" -o "sources/var_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '


