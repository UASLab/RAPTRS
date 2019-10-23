/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include <stdint.h>
#include <time.h>

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
#define _Bool bool
extern "C" {
#endif

uint64_t millis();
uint64_t micros();
uint64_t nanos();

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
}
#endif
