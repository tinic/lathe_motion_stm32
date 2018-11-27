# Makefile for STM32F103C8
#

################################################################################
#####
##### PROJECT
#####
##### The name of the project used as the build target name
#####
#####   -   Explicitly configure project name;
#####   -   Create project name from name of parent directory;


PROJECT         =  lathe_motion
#PROJECT         = $(lastword $(subst /, ,$(shell cd ..;pwd)))

################################################################################
#####
##### Project target
#####

#ARMCPU		= cortex-m3
#STM32MCU	= STM32F103xB
#CMSISDEVINC	= STM32F1xx
#CMSISSTARTUPS 	= startup_stm32f103xb.s
#CMSISLINKLD 	= STM32F103XB_FLASH.ld

ARMCPU		= cortex-m7
STM32MCU	= STM32F767xx
CMSISDEVINC	= STM32F7xx
CMSISSTARTUPS 	= startup_stm32f767xx.s
CMSISLINKLD 	= STM32F767XX_FLASH.ld

################################################################################
#####
##### include master makefile
#####

## kludge to handle directory names with spaces
SPACECHAR 	 = 
SPACECHAR 	+=
BASEPATH 	 = $(subst $(SPACECHAR),\ ,$(shell cd ..;pwd))

include $(BASEPATH)/master.mk
