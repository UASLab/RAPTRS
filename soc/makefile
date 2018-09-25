#
# MAKEFILE
#
# Brian R Taylor
# brian.taylor@bolderflight.com
#
# Copyright (c) 2018 Bolder Flight Systems
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# and associated documentation files (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or
# substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

# output targets
TARGET_FLIGHT := flightcode
TARGET_DATALOG := datalog-server
TARGET_CAL := calibrate-surf
TARGET_FILT := filter-test
TARGET_TELEM := telemetry-server

# compiler
CXX := arm-linux-gnueabihf-g++-8

# cxx flags
override CXXFLAGS += -std=c++17 -O3 -Wno-psabi -I includes/

# directory structure
OBJDIR := obj
BINDIR := bin
SRCDIR := src

# flight code objects
OBJECTS_FLIGHT := \
ins-functions.o \
airdata-functions.o \
allocation-functions.o \
control-algorithms.o \
control-functions.o \
excitation-waveforms.o \
filter-algorithms.o \
filter-functions.o \
flow-control-functions.o \
general-functions.o \
generic-function.o \
power.o \
uNavINS.o \
AirData.o \
telemetry.o \
datalog.o \
effector.o \
excitation.o \
control.o \
mission.o \
sensor-processing.o \
fmu.o \
configuration.o \
definition-tree.o \
flightcode.o \
netBuffer.o \
netChannel.o \
netChat.o \
netSocket.o \
strutils.o \
telnet.o

# datalog server objects
OBJECTS_DATALOG := \
definition-tree.o \
datalog.o \
datalog-server.o

# calibration objects
OBJECTS_CAL := \
ins-functions.o \
airdata-functions.o \
allocation-functions.o \
control-algorithms.o \
control-functions.o \
excitation-waveforms.o \
filter-algorithms.o \
filter-functions.o \
flow-control-functions.o \
power.o \
general-functions.o \
generic-function.o \
uNavINS.o \
AirData.o \
datalog.o \
effector.o \
excitation.o \
control.o \
mission.o \
sensor-processing.o \
fmu.o \
configuration.o \
definition-tree.o \
inclinometer.o \
calibrate-surf.o

OBJECTS_FILT := \
filter-algorithms.o \
filter-test.o

OBJECTS_TELEM := \
telemetry.o \
telemetry-server.o

# add prefix to objects
OBJS_FLIGHT := $(addprefix $(OBJDIR)/,$(OBJECTS_FLIGHT))
OBJS_DATALOG := $(addprefix $(OBJDIR)/,$(OBJECTS_DATALOG))
OBJS_CAL := $(addprefix $(OBJDIR)/,$(OBJECTS_CAL))
OBJS_FILT := $(addprefix $(OBJDIR)/,$(OBJECTS_FILT))
OBJS_TELEM := $(addprefix $(OBJDIR)/,$(OBJECTS_TELEM))

# rules
all: flightcode datalog-server telemetry-server calibrate-surf filter-test | $(OBJDIR)
flightcode: $(addprefix $(BINDIR)/,$(TARGET_FLIGHT))
datalog-server: $(addprefix $(BINDIR)/,$(TARGET_DATALOG))
calibrate-surf: $(addprefix $(BINDIR)/,$(TARGET_CAL))
filter-test: $(addprefix $(BINDIR)/,$(TARGET_FILT))
telemetry-server: $(addprefix $(BINDIR)/,$(TARGET_TELEM))

$(addprefix $(BINDIR)/,$(TARGET_FLIGHT)): $(OBJS_FLIGHT) | $(BINDIR)
	@ echo
	@ echo "Building flight code..."
	@ echo
	$(CXX) $(CXXFLAGS) -o $(addprefix $(BINDIR)/,$(TARGET_FLIGHT)) $(OBJS_FLIGHT)
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""

$(addprefix $(BINDIR)/,$(TARGET_DATALOG)): $(OBJS_DATALOG) | $(BINDIR)
	@ echo
	@ echo "Building datalog server..."
	@ echo
	$(CXX) $(CXXFLAGS) -o $(addprefix $(BINDIR)/,$(TARGET_DATALOG)) $(OBJS_DATALOG)
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""

$(addprefix $(BINDIR)/,$(TARGET_TELEM)): $(OBJS_TELEM) | $(BINDIR)
	@ echo
	@ echo "Building telemetry server..."
	@ echo
	$(CXX) $(CXXFLAGS) -o $(addprefix $(BINDIR)/,$(TARGET_TELEM)) $(OBJS_TELEM)
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""

$(addprefix $(BINDIR)/,$(TARGET_CAL)): $(OBJS_CAL) | $(BINDIR)
	@ echo
	@ echo "Building surface calibration..."
	@ echo
	$(CXX) $(CXXFLAGS) -o $(addprefix $(BINDIR)/,$(TARGET_CAL)) $(OBJS_CAL)
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""

$(addprefix $(BINDIR)/,$(TARGET_FILT)): $(OBJS_FILT) | $(BINDIR)
	@ echo
	@ echo "Building filter test..."
	@ echo
	$(CXX) $(CXXFLAGS) -o $(addprefix $(BINDIR)/,$(TARGET_FILT)) $(OBJS_FILT)
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""


$(OBJS_FLIGHT): $(addprefix $(OBJDIR)/,%.o): $(addprefix $(SRCDIR)/,%.cc) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(addprefix $(OBJDIR)/,datalog-server.o): $(addprefix $(SRCDIR)/,datalog-server.cc) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(addprefix $(OBJDIR)/,telemetry-server.o): $(addprefix $(SRCDIR)/,telemetry-server.cc) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(addprefix $(OBJDIR)/,inclinometer.o): $(addprefix $(SRCDIR)/,inclinometer.cc) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(addprefix $(OBJDIR)/,calibrate-surf.o): $(addprefix $(SRCDIR)/,calibrate-surf.cc) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(addprefix $(OBJDIR)/,filter-test.o): $(addprefix $(SRCDIR)/,filter-test.cc) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR):
	mkdir $(OBJDIR)

$(BINDIR):
	mkdir $(BINDIR)

# clean targets and objects
.PHONY: clean
clean:
	-rm $(addprefix $(BINDIR)/,$(TARGET_FLIGHT)) $(addprefix $(BINDIR)/,$(TARGET_DATALOG)) $(addprefix $(BINDIR)/,$(TARGET_CAL)) $(addprefix $(BINDIR)/,$(TARGET_FILT)) $(addprefix $(BINDIR)/,$(TARGET_TELEM)) $(OBJS_FLIGHT) $(addprefix $(OBJDIR)/,datalog-server.o) $(addprefix $(OBJDIR)/,telemetry-server.o) $(addprefix $(OBJDIR)/,calibrate-surf.o) $(addprefix $(OBJDIR)/,inclinometer.o) $(addprefix $(OBJDIR)/,filter-test.o)
