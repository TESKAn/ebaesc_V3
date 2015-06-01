################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../main.c" \

C_SRCS += \
../main.c \

OBJS += \
./main_c.obj \

OBJS_QUOTED += \
"./main_c.obj" \

C_DEPS += \
./main_c.d \

C_DEPS_QUOTED += \
"./main_c.d" \

OBJS_OS_FORMAT += \
./main_c.obj \


# Each subdirectory must supply rules for building sources it contributes
main_c.obj: ../main.c
	@echo 'Building file: $<'
	@echo 'Executing target #46 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"main.args" -o "./main_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

main_c.d: ../main.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '


