/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#include "allocation-functions.h"

void PseudoInverseAllocation::Configure(const rapidjson::Value& Config,std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Inputs", &input_nodes, &InputKeys_);
  LoadOutput(Config, SystemName, "Outputs", &uCmd_nodes);

  // Control Effectiveness
  LoadVal(Config, "Effectiveness", &Effectiveness);

  // Resize the I/O vectors
  numIn = input_nodes.size();
  Objectives.resize(numIn,1);

  numOut = uCmd_nodes.size();
  uCmd.resize(numOut,1);

  // Limits
  Min.resize(numOut);
  Min.setConstant(numOut, std::numeric_limits<float>::lowest());

  LoadVal(Config, "Min", &Min);

  Max.resize(numOut);
  Max.setConstant(numOut, std::numeric_limits<float>::max());

  LoadVal(Config, "Max", &Max);
}

void PseudoInverseAllocation::Initialize() {}
bool PseudoInverseAllocation::Initialized() {return true;}

void PseudoInverseAllocation::Run(Mode mode) {
  // grab inputs
  for (size_t i=0; i < input_nodes.size(); i++) {
    Objectives(i) = input_nodes[i]->getFloat();
  }

  // Pseduo-Inverse solver using singular value decomposition
  // SVD Decomposition based linear algebra solver
  uCmd = Effectiveness.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Objectives); // Jacobi SVD solver

  // saturate output
  for (int i=0; i < uCmd.rows(); i++) {
    if (uCmd(i) <= Min(i)) {
      uCmd(i) = Min(i);
    } else if (uCmd(i) >= Max(i)) {
      uCmd(i) = Max(i);
    }
    uCmd_nodes[i]->setFloat( uCmd(i) );
  }
}

void PseudoInverseAllocation::Clear() {
  Objectives.resize(0);
  Effectiveness.resize(0,0);
  Min.resize(0);
  Max.resize(0);
  uCmd.resize(0);
  uCmd_nodes.resize(0);
  InputKeys_.clear();
}
