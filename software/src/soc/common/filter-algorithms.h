/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/


#pragma once

#include <algorithm>
#include <vector>

/*
General Filter - Implements a general discrete time filter using the
general filter difference equation. Matches the MATLAB filter function.
*/
class __GeneralFilter {
  public:
    void Configure(std::vector<float> num,std::vector<float> den);
    float Run(float input);
    void Clear();
  private:
    std::vector<float> num_;
    std::vector<float> den_;

    std::vector<float> x_;
    std::vector<float> y_;

};
