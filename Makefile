##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O0 -g -fomit-frame-pointer -falign-functions=16 -D SHELL_CONFIG_FILE
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = -lm
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

 # Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-d16
endif

#
# Architecture or project specific options
##############################################################################
USE_SD_SHELL = TRUE
USE_SERVICE_MODE = TRUE
USE_BLE_MODULE = TRUE
USE_UBLOX_GPS_MODULE = TRUE
SHELL_CONFIG_FILE = TRUE
USE_BNO055_MODULE = TRUE
USE_BMX160_MODULE = TRUE
USE_MICROSD_MODULE = TRUE
USE_WINDSENSOR_MODULE = TRUE
USE_LAG_MODULE = TRUE
USE_ADC_MODULE = TRUE
USE_EEPROM_MODULE = TRUE
USE_MATH_MODULE = TRUE
USE_HMC5883_MODULE = FALSE
USE_HMC6343_MODULE = TRUE

USER_CFLAGS += -DUSE_UBLOX_GPS_MODULE=${USE_UBLOX_GPS_MODULE}\
-DUSE_BNO055_MODULE=${USE_BNO055_MODULE}\
-DUSE_BMX160_MODULE=${USE_BMX160_MODULE}\
-DUSE_HMC5883_MODULE=${USE_HMC5883_MODULE}\
-DUSE_HMC6343_MODULE=${USE_HMC6343_MODULE}\
-DUSE_MICROSD_MODULE=${USE_MICROSD_MODULE}\
-DUSE_WINDSENSOR_MODULE=${USE_WINDSENSOR_MODULE}\
-DUSE_BLE_MODULE=${USE_BLE_MODULE}\
-DUSE_SERVICE_MODE=${USE_SERVICE_MODE}\
-DUSE_EEPROM_MODULE=${USE_EEPROM_MODULE}\
-DUSE_LAG_MODULE=${USE_LAG_MODULE}\
-DUSE_ADC_MODULE=${USE_ADC_MODULE}\
-DUSE_MATH_MODULE=${USE_MATH_MODULE}\
-DCHPRINTF_USE_FLOAT=TRUE

#-DUSE_FATFS_MODULE=${USE_FATFS_MODULE}\


##############################################################################
# Project, target, sources and paths
#

# Define project name here
PROJECT = sd_main

# Target settings.
MCU  = cortex-m4

# Imported source files and paths.
CHIBIOS  := ../ChibiOS
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f7xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F7xx/platform.mk
include ./board/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk
# Other files (optional).
#include $(CHIBIOS)/test/lib/test.mk
#include $(CHIBIOS)/test/rt/rt_test.mk
#include $(CHIBIOS)/test/oslib/oslib_test.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
include $(CHIBIOS)/os/various/shell/shell.mk
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk
#User makefiles
include ./sd_modules/sd_modules.mk
# Define linker script file here
LDSCRIPT= $(STARTUPLD)/STM32F76xxI.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
       $(CHIBIOS)/os/various/syscalls.c \
       exeptions.c \
       main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC)

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

#
# Project, target, sources and paths
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DUSE_SD_SHELL=${USE_SD_SHELL} -DCHPRINTF_USE_FLOAT -DBOOTLOADER_ADDRESS=\"0x1FF00000\"

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -L./sd_modules/bmx160/bsx_lite/Lib/libalgobsx3_CortexM4F -lalgobsx
#ULIBS =

#
# End of user section
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

#
# Common rules
##############################################################################

##############################################################################
# Custom rules
#

#
# Custom rules
##############################################################################
