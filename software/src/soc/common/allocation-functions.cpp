/*
allocation-functions.cc
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
