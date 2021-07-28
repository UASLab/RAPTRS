#******************************************************************************
# Build Library
#
# Copyright (C) 2021 Systems Technology, Inc.
#
#*****************************************************************************

include sources.mk

# Architecture Specific Flags



# Compile Defines
#CXX = arm-linux-gnueabihf-g++-8 -Wall -O3 -Wno-psabi
#STILIB = sti_soc.a

CXX = g++ -Wall -O3 -Wno-psabi
STILIB = sti_sim.a

OBJS = $(SRCS:.cpp=.o)


# Build any object file
%.o : %.cpp
	$(CXX) -c $(SRCS) 


.PHONY: build
build: all

.PHONY: all
all: $(STILIB)

# Construct Library
$(STILIB) : $(OBJS)
	ar csrv $(STILIB) $(OBJS)

.PHONY: clean
clean:
	rm -f $(STILIB) $(OBJS) 
