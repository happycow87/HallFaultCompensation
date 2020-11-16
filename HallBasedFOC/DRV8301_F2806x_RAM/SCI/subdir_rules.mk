################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
SCI/%.obj: ../SCI/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1010/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/bin/cl2000" -v28 -ml --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/Users/Randy/HallFaultCompensation/HallBasedFOC/SCI" --include_path="C:/ti/ccs1010/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/include" --include_path="C:/ti/controlSuite/device_support/f2806x/v110/F2806x_headers/include" --include_path="C:/ti/controlSuite/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSuite/device_support/f2806x/v110/F2806x_common/include" --include_path="C:/ti/controlSuite/development_kits/~SupportFiles/F2806x_headers" --include_path="C:/ti/controlSuite/libs/app_libs/motor_control/math_blocks/v3.1" --define="_DEBUG" --define="DRV8301" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="SCI/$(basename $(<F)).d_raw" --obj_directory="SCI" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


