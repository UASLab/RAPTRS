# The name of your project (used to name the compiled .hex file)
TARGET = $(notdir $(CURDIR))

# Set CPU core speed
TEENSY_CORE_SPEED = 240000000

# configurable options
OPTIONS = -DUSB_SERIAL -DLAYOUT_US_ENGLISH

# directory to build objects in
BUILDDIR = $(abspath $(CURDIR)/obj)

# directory to build executables in
BINPATH = bin

# path location for Teensy 3 core
COREPATH = core

# path location for Arduino libraries
LIBRARYPATH = libraries

# path location for Teensy Loader, teensy_post_compile and teensy_reboot
TOOLSPATH = $(CURDIR)/tools

# path location for the arm-none-eabi compiler
COMPILERPATH = /usr/bin

UNIX_TIME = $(shell date '+%s')

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -Os  -Wno-psabi -mthumb -ffunction-sections -fdata-sections -nostdlib -MMD $(OPTIONS) -DTEENSYDUINO=138 -DF_CPU=$(TEENSY_CORE_SPEED) -Isrc -I$(COREPATH)

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS =

# linker options
LDFLAGS = -O -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=$(UNIX_TIME)

# additional libraries to link
LIBS = -lm

# compiler options
CPPFLAGS += -D__MK66FX1M0__ -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDSCRIPT = $(COREPATH)/mk66fx1m0.ld
LDFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T$(LDSCRIPT)
LIBS += -larm_cortexM4lf_math -L$(abspath $(CURDIR)/tools)

# set arduino define if given
ARDUINO = 10803
CPPFLAGS += -DARDUINO=$(ARDUINO)


# names for the compiler programs
CC = $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CXX = $(abspath $(COMPILERPATH))/arm-none-eabi-g++
OBJCOPY = $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
SIZE = $(abspath $(COMPILERPATH))/arm-none-eabi-size

# automatically create lists of the sources and objects
LC_FILES := $(wildcard $(LIBRARYPATH)/*/*.c)
LCPP_FILES := $(wildcard $(LIBRARYPATH)/*/*.cpp)
TC_FILES := $(wildcard $(COREPATH)/*.c)
TCPP_FILES := $(wildcard $(COREPATH)/*.cpp)
C_FILES := $(wildcard src/*.c)
CPP_FILES := $(wildcard src/*.cpp)
INO_FILES := $(wildcard src/*.ino)

# include paths for libraries
L_INC := $(foreach lib,$(filter %/, $(wildcard $(LIBRARYPATH)/*/)), -I$(lib))

SOURCES := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(INO_FILES:.ino=.o) $(TC_FILES:.c=.o) $(TCPP_FILES:.cpp=.o) $(LC_FILES:.c=.o) $(LCPP_FILES:.cpp=.o)
OBJS := $(foreach src,$(SOURCES), $(BUILDDIR)/$(src))

all: hex

build: $(BINPATH)/$(TARGET).elf

hex: $(BINPATH)/$(TARGET).hex

post_compile: $(BINPATH)/$(TARGET).hex
	@$(abspath $(TOOLSPATH))/teensy_post_compile -file=$(BINPATH)/$(TARGET) -path=$(CURDIR) -tools="$(abspath $(TOOLSPATH))"

reboot:
	@-$(abspath $(TOOLSPATH))/teensy_reboot

upload: post_compile reboot

$(BUILDDIR)/%.o: %.c
	@echo -e "[CC]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.cpp
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.ino
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -x c++ -include Arduino.h -c "$<"

$(BINPATH)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo -e "[LD]\t$@"
	@mkdir -p "$(dir $@)"
	@$(CC) $(LDFLAGS) -o "$@" $(OBJS) $(LIBS)

%.hex: %.elf
	@echo -e "[HEX]\t$@"
	@$(SIZE) "$<"
	@$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"

# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	@echo Cleaning...
	@rm -rf "$(BUILDDIR)"
	@rm -f $(BINPATH)/$(TARGET).elf $(BINPATH)/$(TARGET).hex
