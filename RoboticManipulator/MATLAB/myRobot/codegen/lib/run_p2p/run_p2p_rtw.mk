###########################################################################
## Makefile generated for component 'run_p2p'. 
## 
## Makefile     : run_p2p_rtw.mk
## Generated on : Sun Jul 20 23:36:42 2025
## Final product: ./run_p2p.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file
# MODELLIB                Static library target

PRODUCT_NAME              = run_p2p
MAKEFILE                  = run_p2p_rtw.mk
MATLAB_ROOT               = C:/PROGRA~1/MATLAB/R2024b
MATLAB_BIN                = C:/PROGRA~1/MATLAB/R2024b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
START_DIR                 = D:/6R_robotic_arm/myRobot
TGT_FCN_LIB               = ISO_C
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = ../../..
COMPILER_COMMAND_FILE     = run_p2p_rtw_comp.rsp
CMD_FILE                  = run_p2p_rtw.rsp
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
MODELLIB                  = run_p2p.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU Tools for ARM Embedded Processors
# Supported Version(s):    
# ToolchainInfo Version:   2024b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# TARGET_LOAD_CMD_ARGS
# TARGET_LOAD_CMD
# MW_GNU_ARM_TOOLS_PATH
# FDATASECTIONS_FLG

#-----------
# MACROS
#-----------

LIBGCC                    = ${shell $(MW_GNU_ARM_TOOLS_PATH)/arm-none-eabi-gcc ${CFLAGS} -print-libgcc-file-name}
LIBC                      = ${shell $(MW_GNU_ARM_TOOLS_PATH)/arm-none-eabi-gcc ${CFLAGS} -print-file-name=libc.a}
LIBM                      = ${shell $(MW_GNU_ARM_TOOLS_PATH)/arm-none-eabi-gcc ${CFLAGS} -print-file-name=libm.a}
PRODUCT_NAME_WITHOUT_EXTN = $(basename $(PRODUCT))
PRODUCT_BIN               = $(PRODUCT_NAME_WITHOUT_EXTN).bin
PRODUCT_HEX               = $(PRODUCT_NAME_WITHOUT_EXTN).hex
CPFLAGS                   = -O binary
SHELL                     = %SystemRoot%/system32/cmd.exe

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: GNU ARM Assembler
AS_PATH = $(MW_GNU_ARM_TOOLS_PATH)
AS = "$(AS_PATH)/arm-none-eabi-gcc"

# C Compiler: GNU ARM C Compiler
CC_PATH = $(MW_GNU_ARM_TOOLS_PATH)
CC = "$(CC_PATH)/arm-none-eabi-gcc"

# Linker: GNU ARM Linker
LD_PATH = $(MW_GNU_ARM_TOOLS_PATH)
LD = "$(LD_PATH)/arm-none-eabi-g++"

# C++ Compiler: GNU ARM C++ Compiler
CPP_PATH = $(MW_GNU_ARM_TOOLS_PATH)
CPP = "$(CPP_PATH)/arm-none-eabi-g++"

# C++ Linker: GNU ARM C++ Linker
CPP_LD_PATH = $(MW_GNU_ARM_TOOLS_PATH)
CPP_LD = "$(CPP_LD_PATH)/arm-none-eabi-g++"

# Archiver: GNU ARM Archiver
AR_PATH = $(MW_GNU_ARM_TOOLS_PATH)
AR = "$(AR_PATH)/arm-none-eabi-ar"

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Binary Converter: Binary Converter
OBJCOPYPATH = $(MW_GNU_ARM_TOOLS_PATH)
OBJCOPY = "$(OBJCOPYPATH)/arm-none-eabi-objcopy"

# Hex Converter: Hex Converter
OBJCOPYPATH = $(MW_GNU_ARM_TOOLS_PATH)
OBJCOPY = "$(OBJCOPYPATH)/arm-none-eabi-objcopy"

# Executable Size: Executable Size
EXESIZEPATH = $(MW_GNU_ARM_TOOLS_PATH)
EXESIZE = "$(EXESIZEPATH)/arm-none-eabi-size"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%\bin\win64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @del /f/q
ECHO                = @echo
MV                  = @move
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = ruvs
ASFLAGS              = -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -Wall \
                       -x assembler-with-cpp \
                       $(ASFLAGS_ADDITIONAL) \
                       $(DEFINES) \
                       $(INCLUDES) \
                       -c
OBJCOPYFLAGS_BIN     = -O binary $(PRODUCT) $(PRODUCT_BIN)
CFLAGS               = $(FDATASECTIONS_FLG) \
                       -Wall \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -c \
                       -O3
CPPFLAGS             = -std=gnu++14 \
                       -fno-rtti \
                       -fno-exceptions \
                       $(FDATASECTIONS_FLG) \
                       -Wall \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -c \
                       -O3
CPP_LDFLAGS          = -Wl,--gc-sections \
                       -Wl,-Map="$(PRODUCT_NAME).map"
CPP_SHAREDLIB_LDFLAGS  =
DOWNLOAD_FLAGS       =
EXESIZE_FLAGS        = $(PRODUCT)
EXECUTE_FLAGS        =
OBJCOPYFLAGS_HEX     = -O ihex $(PRODUCT) $(PRODUCT_HEX)
LDFLAGS              = -Wl,--gc-sections \
                       -Wl,-Map="$(PRODUCT_NAME).map"
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    =



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./run_p2p.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -Dccd_EXPORTS -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
DEFINES_CUSTOM = 
DEFINES_SKIPFORSIL = -DUSE_STDPERIPH_DRIVER -DUSE_STM32F4_DISCOVERY -DSTM32F4XX -DARM_MATH_CM4=1 -D__FPU_PRESENT=1 -D__FPU_USED=1U -DHSE_VALUE=8000000 -DNULL=0 -D__START=_start -DEXIT_FAILURE=1 -DEXTMODE_DISABLEPRINTF -DEXTMODE_DISABLETESTING -DEXTMODE_DISABLE_ARGS_PROCESSING=1 -DSTACK_SIZE=200000
DEFINES_STANDARD = -DMODEL=run_p2p

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_ccd.c $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_mpr.c $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_polytope.c $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_vec3.c $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_CollisionGeometry.cpp $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_api.cpp $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_ccdExtensions.cpp $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_checkCollision.cpp $(START_DIR)/codegen/lib/run_p2p/coder_posix_time.c $(START_DIR)/codegen/lib/run_p2p/run_p2p_data.c $(START_DIR)/codegen/lib/run_p2p/rt_nonfinite.c $(START_DIR)/codegen/lib/run_p2p/rtGetNaN.c $(START_DIR)/codegen/lib/run_p2p/rtGetInf.c $(START_DIR)/codegen/lib/run_p2p/run_p2p_initialize.c $(START_DIR)/codegen/lib/run_p2p/run_p2p_terminate.c $(START_DIR)/codegen/lib/run_p2p/run_p2p.c $(START_DIR)/codegen/lib/run_p2p/rand.c $(START_DIR)/codegen/lib/run_p2p/RigidBody.c $(START_DIR)/codegen/lib/run_p2p/rigidBodyJoint.c $(START_DIR)/codegen/lib/run_p2p/CollisionSet.c $(START_DIR)/codegen/lib/run_p2p/eye.c $(START_DIR)/codegen/lib/run_p2p/eul2quat.c $(START_DIR)/codegen/lib/run_p2p/minjerkpolytraj.c $(START_DIR)/codegen/lib/run_p2p/find.c $(START_DIR)/codegen/lib/run_p2p/xnrm2.c $(START_DIR)/codegen/lib/run_p2p/qrsolve.c $(START_DIR)/codegen/lib/run_p2p/norm.c $(START_DIR)/codegen/lib/run_p2p/constructM.c $(START_DIR)/codegen/lib/run_p2p/solvePoly.c $(START_DIR)/codegen/lib/run_p2p/linspace.c $(START_DIR)/codegen/lib/run_p2p/polyder.c $(START_DIR)/codegen/lib/run_p2p/minsnappolytraj.c $(START_DIR)/codegen/lib/run_p2p/quinticpolytraj.c $(START_DIR)/codegen/lib/run_p2p/cubicpolytraj.c $(START_DIR)/codegen/lib/run_p2p/trapveltraj.c $(START_DIR)/codegen/lib/run_p2p/ppval.c $(START_DIR)/codegen/lib/run_p2p/all.c $(START_DIR)/codegen/lib/run_p2p/rottraj.c $(START_DIR)/codegen/lib/run_p2p/slerp.c $(START_DIR)/codegen/lib/run_p2p/trvec2tform.c $(START_DIR)/codegen/lib/run_p2p/quat2tform.c $(START_DIR)/codegen/lib/run_p2p/ikine_myRobot.c $(START_DIR)/codegen/lib/run_p2p/power.c $(START_DIR)/codegen/lib/run_p2p/atan2.c $(START_DIR)/codegen/lib/run_p2p/abs.c $(START_DIR)/codegen/lib/run_p2p/wrapTo2Pi.c $(START_DIR)/codegen/lib/run_p2p/rotm2eul.c $(START_DIR)/codegen/lib/run_p2p/any.c $(START_DIR)/codegen/lib/run_p2p/sum.c $(START_DIR)/codegen/lib/run_p2p/repmat.c $(START_DIR)/codegen/lib/run_p2p/unique.c $(START_DIR)/codegen/lib/run_p2p/sortLE.c $(START_DIR)/codegen/lib/run_p2p/RigidBodyTreeUtils.c $(START_DIR)/codegen/lib/run_p2p/rigidBodyTree.c $(START_DIR)/codegen/lib/run_p2p/mldivide.c $(START_DIR)/codegen/lib/run_p2p/handle.c $(START_DIR)/codegen/lib/run_p2p/eml_rand_mt19937ar_stateful.c $(START_DIR)/codegen/lib/run_p2p/nthroot.c $(START_DIR)/codegen/lib/run_p2p/mod.c $(START_DIR)/codegen/lib/run_p2p/importrobot.c $(START_DIR)/codegen/lib/run_p2p/xzgetrf.c $(START_DIR)/codegen/lib/run_p2p/xgeqp3.c $(START_DIR)/codegen/lib/run_p2p/xgerc.c $(START_DIR)/codegen/lib/run_p2p/histcounts.c $(START_DIR)/codegen/lib/run_p2p/log.c $(START_DIR)/codegen/lib/run_p2p/sqrt.c $(START_DIR)/codegen/lib/run_p2p/asin.c $(START_DIR)/codegen/lib/run_p2p/sort.c $(START_DIR)/codegen/lib/run_p2p/wrapToPi.c $(START_DIR)/codegen/lib/run_p2p/round.c $(START_DIR)/codegen/lib/run_p2p/sortIdx.c $(START_DIR)/codegen/lib/run_p2p/run_p2p_emxutil.c $(START_DIR)/codegen/lib/run_p2p/computePolyCoefAndTimeOfArrival.c $(START_DIR)/codegen/lib/run_p2p/quaternion.c $(START_DIR)/codegen/lib/run_p2p/RigidBodyTreeDynamics.c $(START_DIR)/codegen/lib/run_p2p/run_p2p_rtwutil.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = ccd_ccd.o ccd_mpr.o ccd_polytope.o ccd_vec3.o collisioncodegen_CollisionGeometry.o collisioncodegen_api.o collisioncodegen_ccdExtensions.o collisioncodegen_checkCollision.o coder_posix_time.o run_p2p_data.o rt_nonfinite.o rtGetNaN.o rtGetInf.o run_p2p_initialize.o run_p2p_terminate.o run_p2p.o rand.o RigidBody.o rigidBodyJoint.o CollisionSet.o eye.o eul2quat.o minjerkpolytraj.o find.o xnrm2.o qrsolve.o norm.o constructM.o solvePoly.o linspace.o polyder.o minsnappolytraj.o quinticpolytraj.o cubicpolytraj.o trapveltraj.o ppval.o all.o rottraj.o slerp.o trvec2tform.o quat2tform.o ikine_myRobot.o power.o atan2.o abs.o wrapTo2Pi.o rotm2eul.o any.o sum.o repmat.o unique.o sortLE.o RigidBodyTreeUtils.o rigidBodyTree.o mldivide.o handle.o eml_rand_mt19937ar_stateful.o nthroot.o mod.o importrobot.o xzgetrf.o xgeqp3.o xgerc.o histcounts.o log.o sqrt.o asin.o sort.o wrapToPi.o round.o sortIdx.o run_p2p_emxutil.o computePolyCoefAndTimeOfArrival.o quaternion.o RigidBodyTreeDynamics.o run_p2p_rtwutil.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fdata-sections -ffunction-sections -fmessage-length=0 -fno-short-wchar -fshort-enums -fno-delete-null-pointer-checks -fomit-frame-pointer -funsigned-char -Wall -Wextra  -Wno-unused-parameter -Wno-missing-field-initializers -Wvla -include stm32f4discovery_wrapper.h
CFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CFLAGS += $(CFLAGS_SKIPFORSIL) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fdata-sections -ffunction-sections -fmessage-length=0 -fno-short-wchar -fshort-enums -fno-delete-null-pointer-checks -fomit-frame-pointer -funsigned-char -Wall -Wextra  -Wno-unused-parameter -Wno-missing-field-initializers -Wvla -include stm32f4discovery_wrapper.h
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS += $(CPPFLAGS_SKIPFORSIL) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs -T"C:\ProgramData\MATLAB\SupportPackages\R2024b\toolbox\target\supportpackages\stm32f4discovery/src/arm-gcc-link.ld"

CPP_LDFLAGS += $(CPP_LDFLAGS_SKIPFORSIL)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs -T"C:\ProgramData\MATLAB\SupportPackages\R2024b\toolbox\target\supportpackages\stm32f4discovery/src/arm-gcc-link.ld"

CPP_SHAREDLIB_LDFLAGS += $(CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL)

#-----------
# Linker
#-----------

LDFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs -T"C:\ProgramData\MATLAB\SupportPackages\R2024b\toolbox\target\supportpackages\stm32f4discovery/src/arm-gcc-link.ld"

LDFLAGS += $(LDFLAGS_SKIPFORSIL)

#---------------------
# MEX C++ Compiler
#---------------------

MEX_CPP_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CPPFLAGS += $(MEX_CPP_Compiler_BASIC)

#-----------------
# MEX Compiler
#-----------------

MEX_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CFLAGS += $(MEX_Compiler_BASIC)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs -T"C:\ProgramData\MATLAB\SupportPackages\R2024b\toolbox\target\supportpackages\stm32f4discovery/src/arm-gcc-link.ld"

SHAREDLIB_LDFLAGS += $(SHAREDLIB_LDFLAGS_SKIPFORSIL)

###########################################################################
## INLINED COMMANDS
###########################################################################


ALL_DEPS:=$(patsubst %.o,%.dep,$(ALL_OBJS))
all:

ifndef DISABLE_GCC_FUNCTION_DATA_SECTIONS
FDATASECTIONS_FLG := -ffunction-sections -fdata-sections
endif



-include codertarget_assembly_flags.mk
-include ../codertarget_assembly_flags.mk
-include ../../codertarget_assembly_flags.mk
-include mw_gnu_arm_tools_path.mk
-include ../mw_gnu_arm_tools_path.mk
-include ../../mw_gnu_arm_tools_path.mk
-include $(ALL_DEPS)


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild postbuild download execute


all : build postbuild
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


postbuild : $(PRODUCT)


download : postbuild


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) @$(CMD_FILE)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : %.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/run_p2p/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/run_p2p/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/run_p2p/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/run_p2p/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/run_p2p/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/run_p2p/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/run_p2p/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ccd_ccd.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_ccd.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ccd_mpr.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_mpr.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ccd_polytope.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_polytope.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ccd_vec3.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_vec3.c
	$(CC) $(CFLAGS) -o "$@" "$<"


collisioncodegen_CollisionGeometry.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_CollisionGeometry.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


collisioncodegen_api.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_api.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


collisioncodegen_ccdExtensions.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_ccdExtensions.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


collisioncodegen_checkCollision.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_checkCollision.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


coder_posix_time.o : $(START_DIR)/codegen/lib/run_p2p/coder_posix_time.c
	$(CC) $(CFLAGS) -o "$@" "$<"


run_p2p_data.o : $(START_DIR)/codegen/lib/run_p2p/run_p2p_data.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/codegen/lib/run_p2p/rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/codegen/lib/run_p2p/rtGetNaN.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/codegen/lib/run_p2p/rtGetInf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


run_p2p_initialize.o : $(START_DIR)/codegen/lib/run_p2p/run_p2p_initialize.c
	$(CC) $(CFLAGS) -o "$@" "$<"


run_p2p_terminate.o : $(START_DIR)/codegen/lib/run_p2p/run_p2p_terminate.c
	$(CC) $(CFLAGS) -o "$@" "$<"


run_p2p.o : $(START_DIR)/codegen/lib/run_p2p/run_p2p.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rand.o : $(START_DIR)/codegen/lib/run_p2p/rand.c
	$(CC) $(CFLAGS) -o "$@" "$<"


RigidBody.o : $(START_DIR)/codegen/lib/run_p2p/RigidBody.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rigidBodyJoint.o : $(START_DIR)/codegen/lib/run_p2p/rigidBodyJoint.c
	$(CC) $(CFLAGS) -o "$@" "$<"


CollisionSet.o : $(START_DIR)/codegen/lib/run_p2p/CollisionSet.c
	$(CC) $(CFLAGS) -o "$@" "$<"


eye.o : $(START_DIR)/codegen/lib/run_p2p/eye.c
	$(CC) $(CFLAGS) -o "$@" "$<"


eul2quat.o : $(START_DIR)/codegen/lib/run_p2p/eul2quat.c
	$(CC) $(CFLAGS) -o "$@" "$<"


minjerkpolytraj.o : $(START_DIR)/codegen/lib/run_p2p/minjerkpolytraj.c
	$(CC) $(CFLAGS) -o "$@" "$<"


find.o : $(START_DIR)/codegen/lib/run_p2p/find.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xnrm2.o : $(START_DIR)/codegen/lib/run_p2p/xnrm2.c
	$(CC) $(CFLAGS) -o "$@" "$<"


qrsolve.o : $(START_DIR)/codegen/lib/run_p2p/qrsolve.c
	$(CC) $(CFLAGS) -o "$@" "$<"


norm.o : $(START_DIR)/codegen/lib/run_p2p/norm.c
	$(CC) $(CFLAGS) -o "$@" "$<"


constructM.o : $(START_DIR)/codegen/lib/run_p2p/constructM.c
	$(CC) $(CFLAGS) -o "$@" "$<"


solvePoly.o : $(START_DIR)/codegen/lib/run_p2p/solvePoly.c
	$(CC) $(CFLAGS) -o "$@" "$<"


linspace.o : $(START_DIR)/codegen/lib/run_p2p/linspace.c
	$(CC) $(CFLAGS) -o "$@" "$<"


polyder.o : $(START_DIR)/codegen/lib/run_p2p/polyder.c
	$(CC) $(CFLAGS) -o "$@" "$<"


minsnappolytraj.o : $(START_DIR)/codegen/lib/run_p2p/minsnappolytraj.c
	$(CC) $(CFLAGS) -o "$@" "$<"


quinticpolytraj.o : $(START_DIR)/codegen/lib/run_p2p/quinticpolytraj.c
	$(CC) $(CFLAGS) -o "$@" "$<"


cubicpolytraj.o : $(START_DIR)/codegen/lib/run_p2p/cubicpolytraj.c
	$(CC) $(CFLAGS) -o "$@" "$<"


trapveltraj.o : $(START_DIR)/codegen/lib/run_p2p/trapveltraj.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ppval.o : $(START_DIR)/codegen/lib/run_p2p/ppval.c
	$(CC) $(CFLAGS) -o "$@" "$<"


all.o : $(START_DIR)/codegen/lib/run_p2p/all.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rottraj.o : $(START_DIR)/codegen/lib/run_p2p/rottraj.c
	$(CC) $(CFLAGS) -o "$@" "$<"


slerp.o : $(START_DIR)/codegen/lib/run_p2p/slerp.c
	$(CC) $(CFLAGS) -o "$@" "$<"


trvec2tform.o : $(START_DIR)/codegen/lib/run_p2p/trvec2tform.c
	$(CC) $(CFLAGS) -o "$@" "$<"


quat2tform.o : $(START_DIR)/codegen/lib/run_p2p/quat2tform.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ikine_myRobot.o : $(START_DIR)/codegen/lib/run_p2p/ikine_myRobot.c
	$(CC) $(CFLAGS) -o "$@" "$<"


power.o : $(START_DIR)/codegen/lib/run_p2p/power.c
	$(CC) $(CFLAGS) -o "$@" "$<"


atan2.o : $(START_DIR)/codegen/lib/run_p2p/atan2.c
	$(CC) $(CFLAGS) -o "$@" "$<"


abs.o : $(START_DIR)/codegen/lib/run_p2p/abs.c
	$(CC) $(CFLAGS) -o "$@" "$<"


wrapTo2Pi.o : $(START_DIR)/codegen/lib/run_p2p/wrapTo2Pi.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rotm2eul.o : $(START_DIR)/codegen/lib/run_p2p/rotm2eul.c
	$(CC) $(CFLAGS) -o "$@" "$<"


any.o : $(START_DIR)/codegen/lib/run_p2p/any.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sum.o : $(START_DIR)/codegen/lib/run_p2p/sum.c
	$(CC) $(CFLAGS) -o "$@" "$<"


repmat.o : $(START_DIR)/codegen/lib/run_p2p/repmat.c
	$(CC) $(CFLAGS) -o "$@" "$<"


unique.o : $(START_DIR)/codegen/lib/run_p2p/unique.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sortLE.o : $(START_DIR)/codegen/lib/run_p2p/sortLE.c
	$(CC) $(CFLAGS) -o "$@" "$<"


RigidBodyTreeUtils.o : $(START_DIR)/codegen/lib/run_p2p/RigidBodyTreeUtils.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rigidBodyTree.o : $(START_DIR)/codegen/lib/run_p2p/rigidBodyTree.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mldivide.o : $(START_DIR)/codegen/lib/run_p2p/mldivide.c
	$(CC) $(CFLAGS) -o "$@" "$<"


handle.o : $(START_DIR)/codegen/lib/run_p2p/handle.c
	$(CC) $(CFLAGS) -o "$@" "$<"


eml_rand_mt19937ar_stateful.o : $(START_DIR)/codegen/lib/run_p2p/eml_rand_mt19937ar_stateful.c
	$(CC) $(CFLAGS) -o "$@" "$<"


nthroot.o : $(START_DIR)/codegen/lib/run_p2p/nthroot.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mod.o : $(START_DIR)/codegen/lib/run_p2p/mod.c
	$(CC) $(CFLAGS) -o "$@" "$<"


importrobot.o : $(START_DIR)/codegen/lib/run_p2p/importrobot.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xzgetrf.o : $(START_DIR)/codegen/lib/run_p2p/xzgetrf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xgeqp3.o : $(START_DIR)/codegen/lib/run_p2p/xgeqp3.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xgerc.o : $(START_DIR)/codegen/lib/run_p2p/xgerc.c
	$(CC) $(CFLAGS) -o "$@" "$<"


histcounts.o : $(START_DIR)/codegen/lib/run_p2p/histcounts.c
	$(CC) $(CFLAGS) -o "$@" "$<"


log.o : $(START_DIR)/codegen/lib/run_p2p/log.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sqrt.o : $(START_DIR)/codegen/lib/run_p2p/sqrt.c
	$(CC) $(CFLAGS) -o "$@" "$<"


asin.o : $(START_DIR)/codegen/lib/run_p2p/asin.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sort.o : $(START_DIR)/codegen/lib/run_p2p/sort.c
	$(CC) $(CFLAGS) -o "$@" "$<"


wrapToPi.o : $(START_DIR)/codegen/lib/run_p2p/wrapToPi.c
	$(CC) $(CFLAGS) -o "$@" "$<"


round.o : $(START_DIR)/codegen/lib/run_p2p/round.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sortIdx.o : $(START_DIR)/codegen/lib/run_p2p/sortIdx.c
	$(CC) $(CFLAGS) -o "$@" "$<"


run_p2p_emxutil.o : $(START_DIR)/codegen/lib/run_p2p/run_p2p_emxutil.c
	$(CC) $(CFLAGS) -o "$@" "$<"


computePolyCoefAndTimeOfArrival.o : $(START_DIR)/codegen/lib/run_p2p/computePolyCoefAndTimeOfArrival.c
	$(CC) $(CFLAGS) -o "$@" "$<"


quaternion.o : $(START_DIR)/codegen/lib/run_p2p/quaternion.c
	$(CC) $(CFLAGS) -o "$@" "$<"


RigidBodyTreeDynamics.o : $(START_DIR)/codegen/lib/run_p2p/RigidBodyTreeDynamics.c
	$(CC) $(CFLAGS) -o "$@" "$<"


run_p2p_rtwutil.o : $(START_DIR)/codegen/lib/run_p2p/run_p2p_rtwutil.c
	$(CC) $(CFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### ASFLAGS = $(ASFLAGS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### OBJCOPYFLAGS_BIN = $(OBJCOPYFLAGS_BIN)"
	@echo "### OBJCOPYFLAGS_HEX = $(OBJCOPYFLAGS_HEX)"
	@echo "### EXESIZE_FLAGS = $(EXESIZE_FLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(subst /,\,$(PRODUCT))
	$(RM) $(subst /,\,$(ALL_OBJS))
	$(RM) *.dep
	$(ECHO) "### Deleted all derived files."


