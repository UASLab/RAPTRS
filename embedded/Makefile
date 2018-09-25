# MAKEFILE
#
# Modified for RAPTRS by:
# Brian R Taylor
# brian.taylor@bolderflight.com
#
# Original Makefile from:
# Teensyduino Core Library
# http://www.pjrc.com/teensy/
# Copyright (c) 2017 PJRC.COM, LLC.
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# 1. The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# 2. If the Software is incorporated into a build system that allows
# selection among a list of target devices, then similar target
# devices manufactured by PJRC.COM must be included in the list of
# target devices and selectable in the same manner.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

#
# ----- QUICK NOTES ON USING THIS -----
# "make" builds node and fmu software into the bin directory
# "make fmu" builds the fmu software
# "make node" builds the node software
# "make upload_fmu" uploads the fmu software
# "make upload_node" uploads the node software
# "make clean" removes the bin and object files
#
# Tools from Teensyduino 1.43. Follow instructions for
# installing udev rules for upload to work:
# https://www.pjrc.com/teensy/td_download.html
#

# --- Path Locations ---
# core
CORE = core
# common
COMMON = common
# fmu
FMU = fmu
# node
NODE = node
# executables
BIN = bin
# objects
BUILD = obj
# teensy tools
TOOLS = tools
# compiler
COMPILER = tools/arm/bin

# --- Sources and Objects ---
CORE_C_FILES := $(wildcard $(CORE)/*.c)
CORE_CPP_FILES := $(wildcard $(CORE)/*.cpp)
COMMON_C_FILES := $(wildcard $(COMMON)/*.c)
COMMON_CPP_FILES := $(wildcard $(COMMON)/*.cpp)
COMMON_ASM_FILES := $(wildcard $(COMMON)/*.S)
FMU_C_FILES := $(wildcard $(FMU)/*.c)
FMU_CPP_FILES := $(wildcard $(FMU)/*.cpp)
NODE_C_FILES := $(wildcard $(NODE)/*.c)
NODE_CPP_FILES := $(wildcard $(NODE)/*.cpp)

SOURCES_FMU := $(CORE_C_FILES:.c=.o) $(CORE_CPP_FILES:.cpp=.o) $(COMMON_C_FILES:.c=.o) $(COMMON_CPP_FILES:.cpp=.o) $(COMMON_ASM_FILES:.S=.o) $(FMU_C_FILES:.c=.o) $(FMU_CPP_FILES:.cpp=.o)
OBJS_FMU := $(foreach src,$(SOURCES_FMU), $(BUILD)/$(src))

SOURCES_NODE := $(CORE_C_FILES:.c=.o) $(CORE_CPP_FILES:.cpp=.o) $(COMMON_C_FILES:.c=.o) $(COMMON_CPP_FILES:.cpp=.o) $(COMMON_ASM_FILES:.S=.o) $(NODE_C_FILES:.c=.o) $(NODE_CPP_FILES:.cpp=.o)
OBJS_NODE := $(foreach src,$(SOURCES_NODE), $(BUILD)/$(src))

# --- Compiler ---
CC = $(COMPILER)/arm-none-eabi-gcc
CXX = $(COMPILER)/arm-none-eabi-g++
OBJCOPY = $(COMPILER)/arm-none-eabi-objcopy
SIZE = $(COMPILER)/arm-none-eabi-size

# compiler options
CPPFLAGS = -g -ffunction-sections -fdata-sections -nostdlib -MMD -Os -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D__MK66FX1M0__ -DF_CPU=240000000 -DTEENSYDUINO=143 -DARDUINO=10805 -DUSB_SERIAL -DLAYOUT_US_ENGLISH -I$(CORE)
CXXFLAGS = -fno-exceptions -felide-constructors -std=gnu++17 -Wno-psabi -Wno-error=narrowing -fno-rtti
LDSCRIPT = $(CORE)/mk66fx1m0.ld
LDFLAGS =  -O -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=$(shell date '+%s') -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T$(LDSCRIPT)  
LIBS = -larm_cortexM4lf_math -lm -lstdc++ -L$(TOOLS)

# --- Rules ---
all: fmu_hex node_hex display

fmu: fmu_hex display

node: node_hex display

fmu_build: $(BIN)/fmu.elf

fmu_hex: $(BIN)/fmu.hex

node_build: $(BIN)/node.elf

node_hex: $(BIN)/node.hex

post_compile_fmu: $(BIN)/fmu.hex
	@$(abspath $(TOOLS))/teensy_post_compile -file=$(BIN)/fmu -path=$(CURDIR) -tools="$(abspath $(TOOLS))"

post_compile_node: $(BIN)/node.hex
	@$(abspath $(TOOLS))/teensy_post_compile -file=$(BIN)/node -path=$(CURDIR) -tools="$(abspath $(TOOLS))"

reboot:
	@-$(abspath $(TOOLS))/teensy_reboot

upload_fmu: post_compile_fmu reboot

upload_node: post_compile_node reboot

$(BUILD)/%.o: %.c
	@echo -e "[CC]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CC) $(CPPFLAGS) -o "$@" -c "$<"

$(BUILD)/%.o: %.cpp
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o "$@" -c "$<"

$(BUILD)/%.o: %.S
	@echo -e "[S]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o "$@" -c "$<"

$(BIN)/fmu.elf: $(OBJS_FMU) $(LDSCRIPT)
	@echo -e "[LD]\t$@"
	@mkdir -p "$(dir $@)"
	@$(CC) $(LDFLAGS) -o "$@" $(OBJS_FMU) $(LIBS)

$(BIN)/node.elf: $(OBJS_NODE) $(LDSCRIPT)
	@echo -e "[LD]\t$@"
	@mkdir -p "$(dir $@)"
	@$(CC) $(LDFLAGS) -o "$@" $(OBJS_NODE) $(LIBS)

%.hex: %.elf
	@echo -e "[HEX]\t$@"
	@$(SIZE) "$<"
	@$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"

display:
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""

clean:
	@echo Cleaning...
	@rm -rf "$(BUILD)" "$(BIN)"
	@rm -f $(BIN)/fmu.elf $(BIN)/node.elf $(BIN)/fmu.hex $(BIN)/node.hex

