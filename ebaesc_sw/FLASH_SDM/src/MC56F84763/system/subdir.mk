################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(SRC)/MC56F84763/system/arch.c" \

C_SRCS += \
$(SRC)/MC56F84763/system/arch.c \

OBJS += \
./src/MC56F84763/system/arch_c.obj \

OBJS_QUOTED += \
"./src/MC56F84763/system/arch_c.obj" \

C_DEPS += \
./src/MC56F84763/system/arch_c.d \

C_DEPS_QUOTED += \
"./src/MC56F84763/system/arch_c.d" \

OBJS_OS_FORMAT += \
./src/MC56F84763/system/arch_c.obj \


# Each subdirectory must supply rules for building sources it contributes
src/MC56F84763/system/arch_c.obj: $(SRC)/MC56F84763/system/arch.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/MC56F84763/system/arch.args" -o "src/MC56F84763/system/arch_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/MC56F84763/system/arch_c.d: $(SRC)/MC56F84763/system/arch.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '


