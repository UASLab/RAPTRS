/*
filter-functions.cc
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

#include "filter-functions.hxx"

void GeneralFilter::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::vector<float> a,b;
  // get output name
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  // get the input
  if (Config.HasMember("Input")) {
    InputKey_ = Config["Input"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(InputKey_)) {
      config_.Input = DefinitionTreePtr->GetValuePtr<float*>(InputKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input ")+InputKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input not specified in configuration."));
  }
  // get the feedforward coefficients
  if (Config.HasMember("b")) {
    for (size_t i=0; i < Config["b"].Size(); i++) {
      b.push_back(Config["b"][i].GetFloat());
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Numerator coefficients not specified in configuration."));
  }
  // get feedback coefficients, if any
  if (Config.HasMember("a")) {
    for (size_t i=0; i < Config["a"].Size(); i++) {
      a.push_back(Config["a"][i].GetFloat());
    }
  }

  // pointer to log run mode data
  ModeKey_ = RootPath+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Control law mode",true,false);
  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Output,"Control law output",true,false);

  // configure filter
  filter_.Configure(b,a);
}

void GeneralFilter::Initialize() {}

bool GeneralFilter::Initialized() {
  return true;
}

void GeneralFilter::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  data_.Output = filter_.Run(*config_.Input);
}

void GeneralFilter::Clear(DefinitionTree *DefinitionTreePtr) {
  filter_.Clear();
  data_.Mode = kStandby;
  data_.Output = 0.0f;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  InputKey_.clear();
  ModeKey_.clear();
  OutputKey_.clear();
}
