/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#ifndef UTILS_H_
#define UTILS_H_

#include "Vector.h"
#include "Arduino.h"

uint64_t micros_64();
float PolyVal(std::vector<float> &Coefficients, float X);

#endif
