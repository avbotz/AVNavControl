# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-CodeSourcery

# This file was modified from the generated version.

GCC_BIN = 
PROJECT = AVNavControl
SRC_PATH = src/
OBJ_PATH = build/ # not used
OBJ_EXT = .o # not used
OBJECTS = $(SRC_PATH)imu.o \
$(SRC_PATH)pc.o \
$(SRC_PATH)avnavcontrol.o \
$(SRC_PATH)analog.o \
$(SRC_PATH)motor.o \
$(SRC_PATH)pid.o \
$(SRC_PATH)pid_library.o \
$(SRC_PATH)global.o \
$(SRC_PATH)Kalman.o \
$(SRC_PATH)buffer.o

ASSEMBLY = $(SRC_PATH)imu.s \
$(SRC_PATH)pc.s \
$(SRC_PATH)avnavcontrol.s \
$(SRC_PATH)analog.s \
$(SRC_PATH)motor.s \
$(SRC_PATH)pid.s \
$(SRC_PATH)pid_library.s \
$(SRC_PATH)global.s \
$(SRC_PATH)Kalman.s \
$(SRC_PATH)buffer.s

#checks if we're on Windows or *nix
#based on whether the SystemRoot variable
#is set (its the variable to the Windows directory
#so should always be set in windows
# Windows
ifdef SystemRoot
   RM = del /Q
   FixPath = $(subst /,\,$(1))
# Linux or OS X (gcc4mbed won't run on platforms other than these three)
else
   RM = rm -f
   FixPath = $(1)
endif

SYS_OBJECTS = ./export/LPC1768/GCC_CS/startup_LPC17xx.o ./export/LPC1768/GCC_CS/sys.o ./export/LPC1768/GCC_CS/cmsis_nvic.o ./export/LPC1768/GCC_CS/core_cm3.o ./export/LPC1768/GCC_CS/system_LPC17xx.o 
INCLUDE_PATHS = -I./export -I./export/LPC1768 -I./export/LPC1768/GCC_CS -I$(SRC_PATH)
LIBRARY_PATHS = -L./export -L./export/LPC1768 -L./export/LPC1768/GCC_CS -L$(SRC_PATH)
LIBRARIES = -lmbed -lcapi
LINKER_SCRIPT = export/LPC1768/GCC_CS/LPC1768.ld

############################################################################### 
AS = $(GCC_BIN)arm-none-eabi-as
CC = $(GCC_BIN)arm-none-eabi-gcc
CPP = $(GCC_BIN)arm-none-eabi-g++
LD = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy

CPU = -mcpu=cortex-m3 -mthumb
CC_FLAGS = $(CPU) -Os -fno-common -fmessage-length=0 -Wall -fno-exceptions -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections 
ONLY_C_FLAGS = -std=gnu99
ONLY_CPP_FLAGS = -std=gnu++98
CC_SYMBOLS = -DTARGET_LPC1768 -DTOOLCHAIN_GCC_ARM -DNDEBUG -D__CORTEX_M3

LD_FLAGS = -mcpu=cortex-m3 -mthumb -Wl,--gc-sections --specs=nano.specs -u _printf_float -u _scanf_float
LD_SYS_LIBS = -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys



all: $(PROJECT).bin

asm: $(ASSEMBLY)

clean:
	$(RM) $(call FixPath,$(OBJECTS)) $(call FixPath,$(ASSEMBLY)) $(PROJECT).bin $(PROJECT).elf

.s.o:
	$(AS)  $(CC_FLAGS) $(CC_SYMBOLS) -o $@ $<

.c.o:
	$(CC) -c $(CC_FLAGS) $(CC_SYMBOLS) $(ONLY_C_FLAGS)   $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	$(CPP) -c $(CC_FLAGS) $(CC_SYMBOLS) $(ONLY_CPP_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.c.s:
	$(CC) -S $(CC_FLAGS) $(CC_SYMBOLS) $(ONLY_C_FLAGS)   $(INCLUDE_PATHS) -o $@ $<
	
.cpp.s:
	$(CPP) -S $(CC_FLAGS) $(CC_SYMBOLS) $(ONLY_CPP_FLAGS) $(INCLUDE_PATHS) -o $@ $<

$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS)
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(LIBRARIES) $(LD_SYS_LIBS) $(LIBRARIES) $(LD_SYS_LIBS)

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary $< $@
