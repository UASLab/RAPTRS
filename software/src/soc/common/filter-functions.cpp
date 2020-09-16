/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "filter-functions.h"

void GeneralFilter::Configure(const rapidjson::Value& Config,std::string SystemPath) {
  // I/O signals
  LoadInput(Config, SystemPath, "Input", &u_node, &InputKey_);
  LoadOutput(Config, SystemPath, "Output", &y_node);

  // Coefficient vectors
  LoadVal(Config, "num", &num, true);
  LoadVal(Config, "den", &den, true);

  // Sample time (required)
  LoadVal(Config, "dt", &dt_);
  if (dt_ > 0.0) {
    UseFixedTimeSample = true;
  } else {
    LoadInput(Config, SystemPath, "Time-Source", &time_node, &TimeKey_);
  }

  // Configure filter
  filter_.Configure(num, den, dt_);
}

void GeneralFilter::Initialize() {}
bool GeneralFilter::Initialized() {
  return true;
}

void GeneralFilter::Run(Mode mode) {
  // sample time computation
  float dt = 0;
  if(UseFixedTimeSample == false) {
    dt = time_node->getFloat();
  } else {
    dt = dt_;
  }

  float u = u_node->getFloat();
  float y = 0.0f;
  filter_.Run(mode, u, dt, &y);
  y_node->setFloat(y);
}

void GeneralFilter::Clear() {
  filter_.Clear();
  y_node->setFloat(0.0f);
  InputKey_.clear();
}
