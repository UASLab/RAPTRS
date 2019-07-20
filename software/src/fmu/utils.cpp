/*
utils.cpp
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

#include "utils.h"

/* 64 bit micros */
uint64_t micros_64() {
  static uint32_t low32, high32;
  uint32_t new_low32 = micros();
  if (new_low32 < low32) {
    high32++;
  }
  low32 = new_low32;
  return((uint64_t) high32 << 32 | low32);
}

/* Polynomial evaluation, returns the value of a polynomial evaluated at X with Coefficients given in in descending powers of the polynomial to be evaluated. */
float PolyVal(std::vector<float> &Coefficients, float X) {
  float Y = Coefficients[0];
  for (uint8_t i = 1; i < Coefficients.size(); i++) {
    Y = Y*X + Coefficients[i];
  }
  return(Y);
}

void HardFail(const char *error) {
  while (true) {
    Serial.println(error);
    delay(1000);
  }
}

