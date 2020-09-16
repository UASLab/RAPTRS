/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/


#pragma once

#include <vector>
#include "generic-function.h"

/*
General Filter - Implements a general discrete time filter using the
general filter difference equation. Matches the MATLAB filter function.
*/
class __GeneralFilter {
  public:
    void Configure(std::vector<float> num, std::vector<float> den, float dt);
    void Run(GenericFunction::Mode mode, float u, float dt, float *y);
    void Clear();
  private:
    uint8_t mode_ = GenericFunction::Mode::kStandby;
    bool initLatch_ = false;

    std::vector<float> num_, den_;
    std::vector<float> x_, y_;

    void InitializeState();
    void OutputEquation(float u);
    void Reset();
};
