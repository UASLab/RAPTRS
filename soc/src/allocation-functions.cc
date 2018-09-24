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

#include "allocation-functions.hxx"

void PseudoInverseAllocation::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  // grab inputs
  if (Config.HasMember("Inputs")) {
    for (size_t i=0; i < Config["Inputs"].Size(); i++) {
      const rapidjson::Value& Input = Config["Inputs"][i];
      InputKeys_.push_back(Input.GetString());
      if (DefinitionTreePtr->GetValuePtr<float*>(InputKeys_.back())) {
        config_.Inputs.push_back(DefinitionTreePtr->GetValuePtr<float*>(InputKeys_.back()));
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Input ")+InputKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Inputs not specified in configuration."));
  }
  // resize objective matrix
  config_.Objectives.resize(config_.Inputs.size(),1);
  // grab outputs
  if (Config.HasMember("Outputs")) {
    // resize output matrix
    data_.uCmd.resize(Config["Outputs"].Size(),1);
    data_.uSat.resize(Config["Outputs"].Size(),1);
    for (size_t i=0; i < Config["Outputs"].Size(); i++) {
      const rapidjson::Value& Output = Config["Outputs"][i];
      std::string OutputName = Output.GetString();

      // pointer to log run mode data
      DefinitionTreePtr->InitMember(RootPath + "/Mode", &data_.Mode, "Run mode", true, false);

      // pointer to log saturation data
      //DefinitionTreePtr->InitMember(RootPath + "/Saturated" + "/" + OutputName, &data_.uSat(i), "Allocation saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit", true, false);

      // pointer to log output
      DefinitionTreePtr->InitMember(RootPath + "/" + OutputName, &data_.uCmd(i), "Allocator output", true, false);
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
  if (Config.HasMember("Limits")) {
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      // resize limit vectors
      config_.LowerLimit.resize(Config["Limits"]["Lower"].Size(),1);
      config_.UpperLimit.resize(Config["Limits"]["Upper"].Size(),1);
      for (size_t i=0; i < Config["Limits"]["Lower"].Size(); i++) {
        config_.LowerLimit(i) = Config["Limits"]["Lower"][i].GetFloat();
      }
      for (size_t i=0; i < Config["Limits"]["Upper"].Size(); i++) {
        config_.UpperLimit(i) = Config["Limits"]["Upper"][i].GetFloat();
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Either upper or lower limit not specified in configuration."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Limits not specified in configuration."));
  }
}

void PseudoInverseAllocation::Initialize() {}
bool PseudoInverseAllocation::Initialized() {return true;}

void PseudoInverseAllocation::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  // grab inputs
  for (size_t i=0; i < config_.Inputs.size(); i++) {
    config_.Objectives(i) = *config_.Inputs[i];
  }

  // Pseduo-Inverse solver using singular value decomposition
  // SVD Decomposition based linear algebra solver
  data_.uCmd = config_.Effectiveness.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(config_.Objectives); // Jacobi SVD solver

  // saturate output
  for (size_t i=0; i < data_.uCmd.rows(); i++) {
    if (data_.uCmd(i) <= config_.LowerLimit(i)) {
      data_.uCmd(i) = config_.LowerLimit(i);
      data_.uSat(i) = -1;
    } else if (data_.uCmd(i) >= config_.UpperLimit(i)) {
      data_.uCmd(i) = config_.UpperLimit(i);
      data_.uSat(i) = 1;
    } else {
      data_.uSat(i) = 0;
    }
  }
}

void PseudoInverseAllocation::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Objectives.resize(0);
  config_.Effectiveness.resize(0,0);
  config_.LowerLimit.resize(0);
  config_.UpperLimit.resize(0);
  data_.Mode = (uint8_t) kStandby;
  data_.uCmd.resize(0);
  data_.uSat.resize(0);
  InputKeys_.clear();
}
