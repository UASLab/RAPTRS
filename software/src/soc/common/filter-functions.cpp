/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "filter-functions.h"

void GeneralFilter::Configure(const rapidjson::Value& Config,std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Input", &u_node, &InputKey_);
  LoadOutput(Config, SystemName, "Output", &y_node);

  // Coefficient vectors
  LoadVal(Config, "num", &num, true);
  LoadVal(Config, "den", &den, true);

  // Configure filter
  filter_.Configure(num, den);
}

void GeneralFilter::Initialize() {}

bool GeneralFilter::Initialized() {
  return true;
}

void GeneralFilter::Run(Mode mode) {
  y_node->setFloat( filter_.Run(u_node->getFloat()) );
}

void GeneralFilter::Clear() {
  filter_.Clear();
  y_node->setFloat(0.0f);
  InputKey_.clear();
}
