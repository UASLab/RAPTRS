/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "millis.h"

static inline struct timespec gettime() {
  struct timespec tcur;
  clock_gettime(CLOCK_MONOTONIC, &tcur);
  return tcur;
}

uint64_t millis()
{
  struct timespec tcur = gettime();
  return tcur.tv_sec * (uint64_t)1000 + tcur.tv_nsec / 1000000;
}

uint64_t micros()
{
  struct timespec tcur = gettime();
  return (uint64_t)tcur.tv_sec * (uint64_t)1000000 + (uint64_t)tcur.tv_nsec / 1000;
}

uint64_t nanos()
{
  struct timespec tcur = gettime();
  return (uint64_t)tcur.tv_sec * (uint64_t)1000000000 + (uint64_t)tcur.tv_nsec;
}
