################################################################################
# Automatically-generated file. Do not edit!
################################################################################

#Progress monitor hint: 51
first : all
-include ../makefile.init

# This file contains definitions of environment variables used in the makefiles and .args files if exist.
-include makefile.local

RM := "$(GNU_Make_Install_DirEnv)/rm" -f

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/support/freemaster/subdir.mk
-include src/MC56F84763/system/subdir.mk
-include src/MC56F84763/peripherals/subdir.mk
-include sources/subdir.mk
-include SystemConfig/subdir.mk
-include EepromDrv/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
56F84763_Build.elf \

EXECUTABLES_QUOTED += \
"56F84763_Build.elf" \

EXECUTABLES_OS_FORMAT += \
56F84763_Build.elf \


# All Target
all: warning-messages explicit-dependencies 56F84763_Build.elf

# Tool invocations
56F84763_Build.elf: $(OBJS) $(USER_OBJS) ../SystemConfig/Internal_PFlash_LDM.cmd
	@echo 'Building target: $@'
	@echo 'Executing target #52 $@'
	@echo 'Invoking: DSC Linker'
	"$(DSC_ToolsDirEnv)/mwld56800e" -o "56F84763_Build.elf" @@"56F84763_Build.args"  $(USER_OBJS_QUOTED)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(S_DEPS_QUOTED) $(CPP_DEPS_QUOTED) $(CC_DEPS_QUOTED) "./*/*.obj" "./*.obj" "./*/*/*/*.obj"  $(S_UPPER_DEPS_QUOTED) $(CXX_DEPS_QUOTED) $(ASM_DEPS_QUOTED) $(ASM_UPPER_DEPS_QUOTED) $(C_UPPER_DEPS_QUOTED) $(C++_DEPS_QUOTED) $(EXECUTABLES_QUOTED) "./*/*.d" "./*.d" "./*/*/*/*.d"  $(ProjDirPath)/FLASH_LDM/56F84763_Build.elf
	-@echo ' '

.PHONY: all clean dependents explicit-dependencies warning-messages
.SECONDARY:

-include ../makefile.targets

explicit-dependencies: ../SystemConfig/Internal_PFlash_LDM.cmd 

warning-messages: 

