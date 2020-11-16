################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
SCI/%.obj: ../SCI/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1010/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --include_path="C:/ti/ccs1010/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/include" --include_path="G:/我的雲端硬碟/MyComputor/LAB/VIVA/libs/math/IQmath/v15c/include" --include_path="G:/我的雲端硬碟/MyComputor/LAB/VIVA/development_kits/~SupportFiles/F2803x_headers" --include_path="G:/我的雲端硬碟/MyComputor/LAB/VIVA/device_support/f2803x/v124/DSP2803x_headers/include" --include_path="G:/我的雲端硬碟/MyComputor/LAB/VIVA/device_support/f2803x/v124/DSP2803x_common/include" --include_path="G:/我的雲端硬碟/MyComputor/LAB/VIVA/libs/app_libs/motor_control/math_blocks/v3.1" --define="DRV8301" -g --diag_warning=225 --abi=coffabi --preproc_with_compile --preproc_dependency="SCI/$(basename $(<F)).d_raw" --obj_directory="SCI" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


