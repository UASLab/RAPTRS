/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "configuration.h"
#include "generic-function.h"
#include "filter-algorithms.h"

#include <Eigen/Dense>

/*
General Filter - Implements a general discrete time filter using the
general filter difference equation.

den[0]y[n] = num[0]x[n]+num[1]x[n-1]+num[2]x[n-2]+...-den[1]y[n-1]-den[2]y[n-2]-...

Example JSON configuration:
{
  "Type": "Filter",
  "Input": "InputPath",
  "Output": "OutputName",
  "num": [X],
  "den": [X]
}
Where:
   * Input gives the full path of the input to the filter
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * den is a vector of denominator coefficients. den[0] scales all den and num coefficients if given.
     Denominator coefficients are optional and, if none are provided, a FIR filter is implemented.
   * num is a vector of numerator coefficients. At least one feedforward coefficient must be given.
The order of the filter is given by the length of the num and den vectors.
*/
class GeneralFilter: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    std::vector<float> num;
    std::vector<float> den;

    ElementPtr u_node;
    ElementPtr y_node;

    __GeneralFilter filter_;

    std::string InputKey_;
};
