/*
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
