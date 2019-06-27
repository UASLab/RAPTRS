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

void PseudoInverseAllocation::Configure(const rapidjson::Value& Config,std::string RootPath) {
  // grab inputs
  if (Config.HasMember("Inputs")) {
    numIn = Config["Inputs"].Size();
    for (size_t i=0; i < numIn; i++) {
      const rapidjson::Value& Input = Config["Inputs"][i];
      InputKeys_.push_back(Input.GetString());
      ElementPtr ele = deftree.getElement(InputKeys_.back());
      if ( ele ) {
        config_.input_nodes.push_back(ele);
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Input ")+InputKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Inputs not specified in configuration."));
  }

  // resize objective matrix
  config_.Objectives.resize(numIn,1);

  // grab outputs
  if (Config.HasMember("Outputs")) {
    // resize output matrix
    numOut = Config["Outputs"].Size();
    data_.uCmd.resize(numOut,1);
    data_.uCmd_nodes.resize(numOut);
    for (size_t i=0; i < numOut; i++) {
      const rapidjson::Value& Output = Config["Outputs"][i];
      std::string OutputName = Output.GetString();

      // pointer to log output
      data_.uCmd_nodes[i] = deftree.initElement(RootPath + "/" + OutputName, "Allocator output", LOG_FLOAT, LOG_NONE);
    }
  } else {
    throw std::runtime_error(std::string("ERROR") + RootPath + std::string(": Outputs not specified in configuration."));
  }

  // grab Effectiveness
  if (Config.HasMember("Effectiveness")) {
    // resize Effectiveness matrix
    config_.Effectiveness.resize(Config["Effectiveness"].Size(), Config["Effectiveness"][0].Size());
    for (size_t m=0; m < Config["Effectiveness"].Size(); m++) {
      for (size_t n=0; n < Config["Effectiveness"][m].Size(); n++) {
        config_.Effectiveness(m,n) = Config["Effectiveness"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Effectiveness not specified in configuration."));
  }

  // grab limits
  config_.Min.resize(numOut);
  config_.Min.setConstant(numOut, std::numeric_limits<float>::lowest());

  if (Config.HasMember("Min")) {
    config_.Min.resize(Config["Min"].Size());
    for (size_t i=0; i < Config["Min"].Size(); i++) {
      config_.Min(i) = Config["Min"][i].GetFloat();
    }
  }

  config_.Max.resize(numOut);
  config_.Max.setConstant(numOut, std::numeric_limits<float>::max());

  if (Config.HasMember("Max")) {
    config_.Max.resize(Config["Max"].Size());
    for (size_t i=0; i < Config["Max"].Size(); i++) {
      config_.Max(i) = Config["Max"][i].GetFloat();
    }
  }

}

void PseudoInverseAllocation::Initialize() {}
bool PseudoInverseAllocation::Initialized() {return true;}

void PseudoInverseAllocation::Run(Mode mode) {
  // grab inputs
  for (size_t i=0; i < config_.input_nodes.size(); i++) {
    config_.Objectives(i) = config_.input_nodes[i]->getFloat();
  }

  // Pseduo-Inverse solver using singular value decomposition
  // SVD Decomposition based linear algebra solver
  data_.uCmd = config_.Effectiveness.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(config_.Objectives); // Jacobi SVD solver

  // saturate output
  for (int i=0; i < data_.uCmd.rows(); i++) {
    if (data_.uCmd(i) <= config_.Min(i)) {
      data_.uCmd(i) = config_.Min(i);
    } else if (data_.uCmd(i) >= config_.Max(i)) {
      data_.uCmd(i) = config_.Max(i);
    }
    data_.uCmd_nodes[i]->setFloat( data_.uCmd(i) );
  }
}

void PseudoInverseAllocation::Clear() {
  config_.Objectives.resize(0);
  config_.Effectiveness.resize(0,0);
  config_.Min.resize(0);
  config_.Max.resize(0);
  data_.uCmd.resize(0);
  data_.uCmd_nodes.resize(0);
  InputKeys_.clear();
}
