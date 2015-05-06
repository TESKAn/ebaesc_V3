################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(FREEMASTER)/freemaster_56F8xxx.c" \
"$(FREEMASTER)/freemaster_appcmd.c" \
"$(FREEMASTER)/freemaster_can.c" \
"$(FREEMASTER)/freemaster_fastrec.c" \
"$(FREEMASTER)/freemaster_protocol.c" \
"$(FREEMASTER)/freemaster_rec.c" \
"$(FREEMASTER)/freemaster_scope.c" \
"$(FREEMASTER)/freemaster_serial.c" \
"$(FREEMASTER)/freemaster_sfio.c" \
"$(FREEMASTER)/freemaster_tsa.c" \

C_SRCS += \
$(FREEMASTER)/freemaster_56F8xxx.c \
$(FREEMASTER)/freemaster_appcmd.c \
$(FREEMASTER)/freemaster_can.c \
$(FREEMASTER)/freemaster_fastrec.c \
$(FREEMASTER)/freemaster_protocol.c \
$(FREEMASTER)/freemaster_rec.c \
$(FREEMASTER)/freemaster_scope.c \
$(FREEMASTER)/freemaster_serial.c \
$(FREEMASTER)/freemaster_sfio.c \
$(FREEMASTER)/freemaster_tsa.c \

OBJS += \
./src/support/freemaster/freemaster_56F8xxx_c.obj \
./src/support/freemaster/freemaster_appcmd_c.obj \
./src/support/freemaster/freemaster_can_c.obj \
./src/support/freemaster/freemaster_fastrec_c.obj \
./src/support/freemaster/freemaster_protocol_c.obj \
./src/support/freemaster/freemaster_rec_c.obj \
./src/support/freemaster/freemaster_scope_c.obj \
./src/support/freemaster/freemaster_serial_c.obj \
./src/support/freemaster/freemaster_sfio_c.obj \
./src/support/freemaster/freemaster_tsa_c.obj \

OBJS_QUOTED += \
"./src/support/freemaster/freemaster_56F8xxx_c.obj" \
"./src/support/freemaster/freemaster_appcmd_c.obj" \
"./src/support/freemaster/freemaster_can_c.obj" \
"./src/support/freemaster/freemaster_fastrec_c.obj" \
"./src/support/freemaster/freemaster_protocol_c.obj" \
"./src/support/freemaster/freemaster_rec_c.obj" \
"./src/support/freemaster/freemaster_scope_c.obj" \
"./src/support/freemaster/freemaster_serial_c.obj" \
"./src/support/freemaster/freemaster_sfio_c.obj" \
"./src/support/freemaster/freemaster_tsa_c.obj" \

C_DEPS += \
./src/support/freemaster/freemaster_56F8xxx_c.d \
./src/support/freemaster/freemaster_appcmd_c.d \
./src/support/freemaster/freemaster_can_c.d \
./src/support/freemaster/freemaster_fastrec_c.d \
./src/support/freemaster/freemaster_protocol_c.d \
./src/support/freemaster/freemaster_rec_c.d \
./src/support/freemaster/freemaster_scope_c.d \
./src/support/freemaster/freemaster_serial_c.d \
./src/support/freemaster/freemaster_sfio_c.d \
./src/support/freemaster/freemaster_tsa_c.d \

C_DEPS_QUOTED += \
"./src/support/freemaster/freemaster_56F8xxx_c.d" \
"./src/support/freemaster/freemaster_appcmd_c.d" \
"./src/support/freemaster/freemaster_can_c.d" \
"./src/support/freemaster/freemaster_fastrec_c.d" \
"./src/support/freemaster/freemaster_protocol_c.d" \
"./src/support/freemaster/freemaster_rec_c.d" \
"./src/support/freemaster/freemaster_scope_c.d" \
"./src/support/freemaster/freemaster_serial_c.d" \
"./src/support/freemaster/freemaster_sfio_c.d" \
"./src/support/freemaster/freemaster_tsa_c.d" \

OBJS_OS_FORMAT += \
./src/support/freemaster/freemaster_56F8xxx_c.obj \
./src/support/freemaster/freemaster_appcmd_c.obj \
./src/support/freemaster/freemaster_can_c.obj \
./src/support/freemaster/freemaster_fastrec_c.obj \
./src/support/freemaster/freemaster_protocol_c.obj \
./src/support/freemaster/freemaster_rec_c.obj \
./src/support/freemaster/freemaster_scope_c.obj \
./src/support/freemaster/freemaster_serial_c.obj \
./src/support/freemaster/freemaster_sfio_c.obj \
./src/support/freemaster/freemaster_tsa_c.obj \


# Each subdirectory must supply rules for building sources it contributes
src/support/freemaster/freemaster_56F8xxx_c.obj: $(FREEMASTER)/freemaster_56F8xxx.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_56F8xxx.args" -o "src/support/freemaster/freemaster_56F8xxx_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_56F8xxx_c.d: $(FREEMASTER)/freemaster_56F8xxx.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_appcmd_c.obj: $(FREEMASTER)/freemaster_appcmd.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_appcmd.args" -o "src/support/freemaster/freemaster_appcmd_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_appcmd_c.d: $(FREEMASTER)/freemaster_appcmd.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_can_c.obj: $(FREEMASTER)/freemaster_can.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_can.args" -o "src/support/freemaster/freemaster_can_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_can_c.d: $(FREEMASTER)/freemaster_can.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_fastrec_c.obj: $(FREEMASTER)/freemaster_fastrec.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_fastrec.args" -o "src/support/freemaster/freemaster_fastrec_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_fastrec_c.d: $(FREEMASTER)/freemaster_fastrec.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_protocol_c.obj: $(FREEMASTER)/freemaster_protocol.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_protocol.args" -o "src/support/freemaster/freemaster_protocol_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_protocol_c.d: $(FREEMASTER)/freemaster_protocol.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_rec_c.obj: $(FREEMASTER)/freemaster_rec.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_rec.args" -o "src/support/freemaster/freemaster_rec_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_rec_c.d: $(FREEMASTER)/freemaster_rec.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_scope_c.obj: $(FREEMASTER)/freemaster_scope.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_scope.args" -o "src/support/freemaster/freemaster_scope_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_scope_c.d: $(FREEMASTER)/freemaster_scope.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_serial_c.obj: $(FREEMASTER)/freemaster_serial.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_serial.args" -o "src/support/freemaster/freemaster_serial_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_serial_c.d: $(FREEMASTER)/freemaster_serial.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_sfio_c.obj: $(FREEMASTER)/freemaster_sfio.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_sfio.args" -o "src/support/freemaster/freemaster_sfio_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_sfio_c.d: $(FREEMASTER)/freemaster_sfio.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

src/support/freemaster/freemaster_tsa_c.obj: $(FREEMASTER)/freemaster_tsa.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: DSC Compiler'
	"$(DSC_ToolsDirEnv)/mwcc56800e" -c @@"src/support/freemaster/freemaster_tsa.args" -o "src/support/freemaster/freemaster_tsa_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

src/support/freemaster/freemaster_tsa_c.d: $(FREEMASTER)/freemaster_tsa.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '


