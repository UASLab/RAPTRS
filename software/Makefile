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
# executables
BIN = bin
# objects
BUILD = obj
# fmu
FMU = src/fmu
# node
NODE = src/node
# architectures
FMU_ARCH = MK66FX1M0V
NODE_ARCH = MK66FX1M0V
SOC_ARCH = AM335X
# common
COMMON = src/common
# arduino libs
ARDUINO_LIBS = src/includes/arduino_libs
# fmu core
FMU_CORE = src/includes/mk66fx1m0v_core
# node core
NODE_CORE = src/includes/mk66fx1m0v_core
# build tools
TOOLS = tools
# fmu compiler
FMU_COMPILER = tools/arm/bin
# node compiler
NODE_COMPILER = tools/arm/bin
# --- Sources and Objects ---
# files
fmu_c_files = $(wildcard $(FMU)/*.c)
fmu_cpp_files = $(wildcard $(FMU)/*.cpp)
node_c_files = $(wildcard $(NODE)/*.c)
node_cpp_files = $(wildcard $(NODE)/*.cpp)
common_c_files = $(wildcard $(COMMON)/*.c)
common_cpp_files = $(wildcard $(COMMON)/*.cpp)
arduino_c_files = $(wildcard $(ARDUINO_LIBS)/*.c)
arduino_cpp_files = $(wildcard $(ARDUINO_LIBS)/*.cpp)
fmu_core_c_files = $(wildcard $(FMU_CORE)/*.c)
fmu_core_cpp_files = $(wildcard $(FMU_CORE)/*.cpp)
node_core_c_files = $(wildcard $(NODE_CORE)/*.c)
node_core_cpp_files = $(wildcard $(NODE_CORE)/*.cpp)
# sources
fmu_src = $(fmu_c_files:.c=.o) $(fmu_cpp_files:.cpp=.o) $(common_src) $(arduino_src) $(fmu_core_src)
node_src = $(node_c_files:.c=.o) $(node_cpp_files:.cpp=.o) $(common_src) $(arduino_src) $(node_core_src)
common_src = $(common_c_files:.c=.o) $(common_cpp_files:.cpp=.o) 
arduino_src = $(arduino_c_files:.c=.o) $(arduino_cpp_files:.cpp=.o) 
fmu_core_src = $(fmu_core_c_files:.c=.o) $(fmu_core_cpp_files:.cpp=.o) 
node_core_src = $(node_core_c_files:.c=.o) $(node_core_cpp_files:.cpp=.o) 
# objects
fmu_obj = $(foreach src,$(fmu_src), $(BUILD)/$(FMU_ARCH)/$(src))
node_obj = $(foreach src,$(node_src), $(BUILD)/$(NODE_ARCH)/$(src))
# --- Compiler ---
FMU_CC = $(FMU_COMPILER)/arm-none-eabi-gcc
FMU_CXX = $(FMU_COMPILER)/arm-none-eabi-g++
FMU_OBJCOPY = $(FMU_COMPILER)/arm-none-eabi-objcopy
FMU_SIZE = $(FMU_COMPILER)/arm-none-eabi-size
NODE_CC = $(NODE_COMPILER)/arm-none-eabi-gcc
NODE_CXX = $(NODE_COMPILER)/arm-none-eabi-g++
NODE_OBJCOPY = $(NODE_COMPILER)/arm-none-eabi-objcopy
NODE_SIZE = $(NODE_COMPILER)/arm-none-eabi-size
# compiler options
FMU_CPPFLAGS = -g -ffunction-sections -fdata-sections -nostdlib -MMD -Os -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D__MK66FX1M0__ -DF_CPU=240000000 -DTEENSYDUINO=144 -DARDUINO=10807 -DUSB_SERIAL -DLAYOUT_US_ENGLISH -I$(COMMON) -I$(ARDUINO_LIBS) -I$(FMU_CORE)
FMU_CXXFLAGS = -fno-exceptions -felide-constructors -std=gnu++17 -Wno-psabi -Wno-error=narrowing -fno-rtti
FMU_LDSCRIPT = $(FMU_CORE)/mk66fx1m0.ld
FMU_LDFLAGS =  -O -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=$(shell date '+%s') -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T$(FMU_LDSCRIPT)  
FMU_LIBS = -larm_cortexM4lf_math -lm -lstdc++ -L$(TOOLS)
NODE_CPPFLAGS = -g -ffunction-sections -fdata-sections -nostdlib -MMD -Os -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D__MK66FX1M0__ -DF_CPU=240000000 -DTEENSYDUINO=144 -DARDUINO=10807 -DUSB_SERIAL -DLAYOUT_US_ENGLISH -I$(COMMON) -I$(ARDUINO_LIBS) -I$(NODE_CORE)
NODE_CXXFLAGS = -fno-exceptions -felide-constructors -std=gnu++17 -Wno-psabi -Wno-error=narrowing -fno-rtti
NODE_LDSCRIPT = $(NODE_CORE)/mk66fx1m0.ld
NODE_LDFLAGS =  -O -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=$(shell date '+%s') -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T$(NODE_LDSCRIPT)  
NODE_LIBS = -larm_cortexM4lf_math -lm -lstdc++ -L$(TOOLS)
# --- Rules ---
all: fmu_hex node_hex display

fmu: fmu_hex display

node: node_hex display

fmu_build: $(BIN)/fmu.elf

node_build: $(BIN)/node.elf

fmu_hex: $(BIN)/fmu.hex

node_hex: $(BIN)/node.hex

post_compile_fmu: $(BIN)/fmu.hex
	@$(abspath $(TOOLS))/teensy_post_compile -file=$(BIN)/fmu -path=$(CURDIR) -tools="$(abspath $(TOOLS))"

post_compile_node: $(BIN)/node.hex
	@$(abspath $(TOOLS))/teensy_post_compile -file=$(BIN)/node -path=$(CURDIR) -tools="$(abspath $(TOOLS))"

reboot:
	@-$(abspath $(TOOLS))/teensy_reboot

upload_fmu: post_compile_fmu reboot

upload_node: post_compile_node reboot

$(BUILD)/$(FMU_ARCH)/%.o: %.c
	@echo -e "[CC]\t$<"
	@mkdir -p "$(dir $@)"
	@$(FMU_CC) $(FMU_CPPFLAGS) -o "$@" -c "$<"

$(BUILD)/$(NODE_ARCH)/%.o: %.c
	@echo -e "[CC]\t$<"
	@mkdir -p "$(dir $@)"
	@$(NODE_CC) $(NODE_CPPFLAGS) -o "$@" -c "$<"

$(BUILD)/$(FMU_ARCH)/%.o: %.cpp
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(FMU_CXX) $(FMU_CPPFLAGS) $(FMU_CXXFLAGS) -o "$@" -c "$<"

$(BUILD)/$(NODE_ARCH)/%.o: %.cpp
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(NODE_CXX) $(NODE_CPPFLAGS) $(NODE_CXXFLAGS) -o "$@" -c "$<"

$(BIN)/fmu.elf: $(fmu_obj) $(FMU_LDSCRIPT)
	@echo -e "[LD]\t$@"
	@mkdir -p "$(dir $@)"
	@$(FMU_CC) $(FMU_LDFLAGS) -o "$@" $(fmu_obj) $(FMU_LIBS)

$(BIN)/node.elf: $(node_obj) $(NODE_LDSCRIPT)
	@echo -e "[LD]\t$@"
	@mkdir -p "$(dir $@)"
	@$(NODE_CC) $(NODE_LDFLAGS) -o "$@" $(node_obj) $(NODE_LIBS)

$(BIN)/fmu.hex: $(BIN)/fmu.elf
	@echo -e "[HEX]\t$@"
	@$(FMU_SIZE) "$<"
	@$(FMU_OBJCOPY) -O ihex -R .eeprom "$<" "$@"

$(BIN)/node.hex: $(BIN)/node.elf
	@echo -e "[HEX]\t$@"
	@$(NODE_SIZE) "$<"
	@$(NODE_OBJCOPY) -O ihex -R .eeprom "$<" "$@"

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