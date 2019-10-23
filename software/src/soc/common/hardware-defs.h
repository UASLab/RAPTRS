/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>

// Software version number
const std::string SoftwareVersion = "0.10.0";

// FMU port, baud, and maximum buffer size
const char FmuPort[] = "/dev/ttyO4";
const uint32_t FmuBaud = 1500000;
const uint32_t kUartBufferMaxSize = 4096;
