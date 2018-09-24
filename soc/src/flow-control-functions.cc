/*
flow-control-functions.cc
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

#include "flow-control-functions.hxx"

void If::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  if (Config.HasMember("Threshold")) {
    config_.Threshold = Config["Threshold"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Threshold value not specified in configuration."));
  }
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
  // pointer to log run mode data
  ModeKey_ = OutputName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Control law mode",true,false);
  // pointer to log command data
  OutputKey_ = OutputName+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Output,"Control law output",true,false);
}

void If::Initialize() {}
bool If::Initialized() {return true;}

void If::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  if (*config_.Input > config_.Threshold) {
    data_.Output = 1;
  } else {
    data_.Output = 0;
  }
}

void If::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Threshold = 0.0f;
  data_.Mode = kStandby;
  data_.Output = 0;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  InputKey_.clear();
  ModeKey_.clear();
  OutputKey_.clear();
}
