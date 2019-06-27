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
  config_.num = num;
  config_.den = den;
  states_.x.resize(config_.num.size());
  states_.y.resize(config_.den.size());
  // scale all den and num by den[0] if available
  if (config_.den.size() > 0) {
    // prevent divide by zero
    if (config_.den[0] != 0.0f) {
      for (size_t i=0; i < config_.num.size(); i++) {
        config_.num[i] = config_.num[i]/config_.den[0];
      }
      for (size_t i=1; i < config_.den.size(); i++) {
        config_.den[i] = config_.den[i]/config_.den[0];
      }
    }
  }
}

float __GeneralFilter::Run(float input) {
  // shift all x and y values to the right 1
  if (states_.x.size()>0) {
    std::rotate(states_.x.data(), states_.x.data()+states_.x.size()-1, states_.x.data()+states_.x.size());
  }
  if (states_.y.size() > 0) {
    std::rotate(states_.y.data(), states_.y.data()+states_.y.size()-1, states_.y.data()+states_.y.size());
  }
  // grab the newest x value
  states_.x[0] = input;
  // apply all num coefficients
  float FeedForward = 0.0f;
  for (size_t i=0; i < config_.num.size(); i++) {
    FeedForward += config_.num[i]*states_.x[i];
  }
  // apply all den coefficients
  float FeedBack = 0.0f;
  for (size_t i=1; i < config_.den.size(); i++) {
    FeedBack += config_.den[i]*states_.y[i];
  }
  // get the output
  data_.Output = FeedForward - FeedBack;
  // grab the newest y value
  if (states_.y.size() > 0) {
    states_.y[0] = data_.Output;
  }
  return data_.Output;
}

void __GeneralFilter::Clear() {
  config_.den.clear();
  config_.num.clear();
  states_.x.clear();
  states_.y.clear();
  data_.Output = 0.0f;
}
