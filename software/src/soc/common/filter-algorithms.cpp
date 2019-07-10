/*
filter-algorithms.cc
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

#include "filter-algorithms.h"

void __GeneralFilter::Configure(std::vector<float> num, std::vector<float> den) {

  x.resize(num.size());
  y.resize(den.size());
  
  // scale all den and num by den[0] if available
  if (den.size() > 0) {
    // prevent divide by zero
    if (den[0] != 0.0f) {
      for (size_t i=0; i < num.size(); i++) {
        num[i] = num[i]/den[0];
      }
      for (size_t i=1; i < den.size(); i++) {
        den[i] = den[i]/den[0];
      }
    }
  }
}

float __GeneralFilter::Run(float input) {
  // shift all x and y values to the right 1
  if (x.size()>0) {
    std::rotate(x.data(), x.data()+x.size()-1, x.data()+x.size());
  }
  if (y.size() > 0) {
    std::rotate(y.data(), y.data()+y.size()-1, y.data()+y.size());
  }

  // grab the newest x value
  x[0] = input;

  // apply all num coefficients
  float FeedForward = 0.0f;
  for (size_t i=0; i < num.size(); i++) {
    FeedForward += num[i]*x[i];
  }

  // apply all den coefficients
  float FeedBack = 0.0f;
  for (size_t i=1; i < den.size(); i++) {
    FeedBack += den[i]*y[i];
  }

  // get the output
  Output = FeedForward - FeedBack;

  // grab the newest y value
  if (y.size() > 0) {
    y[0] = Output;
  }
  return Output;
}

void __GeneralFilter::Clear() {
  den.clear();
  num.clear();
  x.clear();
  y.clear();
  Output = 0.0f;
}
