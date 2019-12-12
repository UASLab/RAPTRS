/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/


#include "filter-algorithms.h"

void __GeneralFilter::Configure(std::vector<float> num, std::vector<float> den) {

  x_.resize(num.size());
  y_.resize(den.size());

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
  num_ = num;
  den_ = den;
}

float __GeneralFilter::Run(float input) {
  // shift all x and y values to the right 1
  if (x_.size()>0) {
    std::rotate(x_.data(), x_.data()+x_.size()-1, x_.data()+x_.size());
  }
  if (y_.size() > 0) {
    std::rotate(y_.data(), y_.data()+y_.size()-1, y_.data()+y_.size());
  }

  // grab the newest x value
  x_[0] = input;

  // apply all num coefficients
  float FeedForward = 0.0f;
  for (size_t i=0; i < num_.size(); i++) {
    FeedForward += num_[i]*x_[i];
  }

  // apply all den coefficients
  float FeedBack = 0.0f;
  for (size_t i=1; i < den_.size(); i++) {
    FeedBack += den_[i]*y_[i];
  }

  // get the output
  float Output = FeedForward - FeedBack;

  // grab the newest y value
  if (y_.size() > 0) {
    y_[0] = Output;
  }
  return Output;
}

void __GeneralFilter::Clear() {
  den_.clear();
  num_.clear();
  x_.clear();
  y_.clear();
}
