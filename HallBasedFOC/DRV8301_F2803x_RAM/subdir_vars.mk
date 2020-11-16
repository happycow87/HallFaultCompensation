################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
C:/ti/controlSUITE/device_support/f2803x/v124/DSP2803x_headers/cmd/DSP2803x_Headers_nonBIOS.cmd \
../F28069_flash.cmd \
../F2806x_RAM_PM_Sensorless.CMD.cmd 

LIB_SRCS += \
C:/ti/controlSUITE/libs/math/IQmath/v15c/lib/IQmath.lib 

ASM_SRCS += \
../DLOG4CHC.asm \
../DSP2803x_usDelay.asm 

CMD_UPPER_SRCS += \
../F28035_RAM_PM_Sensorless.CMD 

C_SRCS += \
../DRV8301_SPI.c \
C:/ti/controlSUITE/device_support/f2803x/v124/DSP2803x_headers/source/DSP2803x_GlobalVariableDefs.c \
../PM_Sensorless-DevInit_F2803x.c \
../PM_Sensorless.c 

C_DEPS += \
./DRV8301_SPI.d \
./DSP2803x_GlobalVariableDefs.d \
./PM_Sensorless-DevInit_F2803x.d \
./PM_Sensorless.d 

OBJS += \
./DLOG4CHC.obj \
./DRV8301_SPI.obj \
./DSP2803x_GlobalVariableDefs.obj \
./DSP2803x_usDelay.obj \
./PM_Sensorless-DevInit_F2803x.obj \
./PM_Sensorless.obj 

ASM_DEPS += \
./DLOG4CHC.d \
./DSP2803x_usDelay.d 

OBJS__QUOTED += \
"DLOG4CHC.obj" \
"DRV8301_SPI.obj" \
"DSP2803x_GlobalVariableDefs.obj" \
"DSP2803x_usDelay.obj" \
"PM_Sensorless-DevInit_F2803x.obj" \
"PM_Sensorless.obj" 

C_DEPS__QUOTED += \
"DRV8301_SPI.d" \
"DSP2803x_GlobalVariableDefs.d" \
"PM_Sensorless-DevInit_F2803x.d" \
"PM_Sensorless.d" 

ASM_DEPS__QUOTED += \
"DLOG4CHC.d" \
"DSP2803x_usDelay.d" 

ASM_SRCS__QUOTED += \
"../DLOG4CHC.asm" \
"../DSP2803x_usDelay.asm" 

C_SRCS__QUOTED += \
"../DRV8301_SPI.c" \
"C:/ti/controlSUITE/device_support/f2803x/v124/DSP2803x_headers/source/DSP2803x_GlobalVariableDefs.c" \
"../PM_Sensorless-DevInit_F2803x.c" \
"../PM_Sensorless.c" 


