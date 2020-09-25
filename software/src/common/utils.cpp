/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
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
