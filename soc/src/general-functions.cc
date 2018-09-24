/*
general-functions.cc
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

#include "general-functions.hxx"
#include <stdio.h>
#include <iostream>

/* Constant class methods, see general-functions.hxx for more information */
void ConstantClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Constant")) {
    config_.Constant = Config["Constant"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Constant value not specified in configuration."));
  }

  // pointer to log command data
  OutputKey_ = OutputName;
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Output,"Control law output",true,false);
}

void ConstantClass::Initialize() {}

bool ConstantClass::Initialized() {return true;}

void ConstantClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  data_.Output = config_.Constant;
}

void ConstantClass::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Constant = 0.0f;
  data_.Mode = kStandby;
  data_.Output = 0.0f;
  DefinitionTreePtr->Erase(OutputKey_);
  OutputKey_.clear();
}

/* Gain class methods, see general-functions.hxx for more information */
void GainClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Gain")) {
    config_.Gain = Config["Gain"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Gain value not specified in configuration."));
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

  if (Config.HasMember("Limits")) {
    config_.SaturateOutput = true;

    // pointer to log saturation data
    SaturatedKey_ = RootPath+"/Saturated";
    DefinitionTreePtr->InitMember(SaturatedKey_,&data_.Saturated,"Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit",true,false);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      config_.UpperLimit = Config["Limits"]["Upper"].GetFloat();
      config_.LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Output,"Control law output",true,false);
}

void GainClass::Initialize() {}
bool GainClass::Initialized() {return true;}

void GainClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  data_.Output = *config_.Input*config_.Gain;
  // saturate command
  if (config_.SaturateOutput) {
    if (data_.Output <= config_.LowerLimit) {
      data_.Output = config_.LowerLimit;
      data_.Saturated = -1;
    } else if (data_.Output >= config_.UpperLimit) {
      data_.Output = config_.UpperLimit;
      data_.Saturated = 1;
    } else {
      data_.Saturated = 0;
    }
  }
}

void GainClass::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Gain = 1.0f;
  config_.LowerLimit = 0.0f;
  config_.UpperLimit = 0.0f;
  config_.SaturateOutput = false;
  data_.Mode = kStandby;
  data_.Output = 0.0f;
  data_.Saturated = 0.0f;
  DefinitionTreePtr->Erase(SaturatedKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  InputKey_.clear();
  SaturatedKey_.clear();
  OutputKey_.clear();
}

/* Sum class methods, see general-functions.hxx for more information */
void SumClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Inputs")) {
    for (size_t i=0; i < Config["Inputs"].Size(); i++) {
      const rapidjson::Value& Input = Config["Inputs"][i];
      InputKeys_.push_back(Input.GetString());

      if (DefinitionTreePtr->GetValuePtr<float*>(InputKeys_.back())) {
        config_.Inputs.push_back(DefinitionTreePtr->GetValuePtr<float*>(InputKeys_.back()));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input ")+InputKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Inputs not specified in configuration."));
  }

  if (Config.HasMember("Limits")) {
    config_.SaturateOutput = true;
    // pointer to log saturation data
    SaturatedKey_ = RootPath+"/Saturated";
    DefinitionTreePtr->InitMember(SaturatedKey_,&data_.Saturated,"Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit",true,false);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      config_.UpperLimit = Config["Limits"]["Upper"].GetFloat();
      config_.LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Output,"Control law output",true,false);
}

void SumClass::Initialize() {}
bool SumClass::Initialized() {return true;}

void SumClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;

  data_.Output = 0.0f;
  for (size_t i=0; i < config_.Inputs.size(); i++) {
    data_.Output += *config_.Inputs[i];
  }

  // saturate command
  if (config_.SaturateOutput) {
    if (data_.Output <= config_.LowerLimit) {
      data_.Output = config_.LowerLimit;
      data_.Saturated = -1;
    } else if (data_.Output >= config_.UpperLimit) {
      data_.Output = config_.UpperLimit;
      data_.Saturated = 1;
    } else {
      data_.Saturated = 0;
    }
  }
}

void SumClass::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Inputs.clear();
  config_.LowerLimit = 0.0f;
  config_.UpperLimit = 0.0f;
  config_.SaturateOutput = false;
  data_.Mode = kStandby;
  data_.Output = 0.0f;
  data_.Saturated = 0.0f;
  DefinitionTreePtr->Erase(SaturatedKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  InputKeys_.clear();
  SaturatedKey_.clear();
  OutputKey_.clear();
}


/* Latch class methods, see general-functions.hxx for more information */
void LatchClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
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

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Output,"Control law output",true,false);
}

void LatchClass::Initialize() {}
bool LatchClass::Initialized() {return true;}

void LatchClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;

  switch(data_.Mode) {
    case GenericFunction::Mode::kEngage: {
      if (initLatch_ == false) {
        initLatch_ = true;
        data_.Output = *config_.Input;
      }
      break;
    }
    default: {
      initLatch_ = false;
    }
  }

}

void LatchClass::Clear(DefinitionTree *DefinitionTreePtr) {
  data_.Mode = kStandby;
  data_.Output = 0.0f;
  DefinitionTreePtr->Erase(OutputKey_);
  InputKey_.clear();
  OutputKey_.clear();
}
