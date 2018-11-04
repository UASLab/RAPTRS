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

#include "general-functions.h"
#include <stdio.h>
#include <iostream>

/* Constant class methods, see general-functions.hxx for more information */
void ConstantClass::Configure(const rapidjson::Value& Config,std::string RootPath) {
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
  data_.output_node = deftree.initElement(OutputKey_,"Control law output", LOG_FLOAT, LOG_NONE);
}

void ConstantClass::Initialize() {}

bool ConstantClass::Initialized() {return true;}

void ConstantClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  data_.output_node->setFloat(config_.Constant);
}

void ConstantClass::Clear() {
  config_.Constant = 0.0f;
  data_.Mode = kStandby;
  data_.output_node->setFloat(0.0f);
  deftree.Erase(OutputKey_);
  OutputKey_.clear();
}

/* Gain class methods, see general-functions.hxx for more information */
void GainClass::Configure(const rapidjson::Value& Config,std::string RootPath) {
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
    config_.input_node = deftree.getElement(InputKey_);
    if ( !config_.input_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input ")+InputKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input not specified in configuration."));
  }

  if (Config.HasMember("Limits")) {
    config_.SaturateOutput = true;

    // pointer to log saturation data
    SaturatedKey_ = RootPath+"/Saturated";
    data_.saturated_node = deftree.initElement(SaturatedKey_, "Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit", LOG_UINT8, LOG_NONE);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      config_.UpperLimit = Config["Limits"]["Upper"].GetFloat();
      config_.LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  data_.output_node = deftree.initElement(OutputKey_, "Control law output", LOG_FLOAT, LOG_NONE);
}

void GainClass::Initialize() {}
bool GainClass::Initialized() {return true;}

void GainClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  float val = config_.input_node->getFloat()*config_.Gain;
  // saturate command
  if (config_.SaturateOutput) {
    if (val <= config_.LowerLimit) {
      val = config_.LowerLimit;
      data_.saturated_node->setInt(-1);
    } else if (val >= config_.UpperLimit) {
      val = config_.UpperLimit;
      data_.saturated_node->setInt(1);
    } else {
      data_.saturated_node->setInt(0);
    }
  }
  data_.output_node->setFloat( val );
}

void GainClass::Clear() {
  config_.Gain = 1.0f;
  config_.LowerLimit = 0.0f;
  config_.UpperLimit = 0.0f;
  config_.SaturateOutput = false;
  data_.Mode = kStandby;
  data_.output_node->setFloat(0.0f);
  data_.saturated_node->setFloat(0.0f);
  deftree.Erase(SaturatedKey_);
  deftree.Erase(OutputKey_);
  InputKey_.clear();
  SaturatedKey_.clear();
  OutputKey_.clear();
}

/* Sum class methods, see general-functions.hxx for more information */
void SumClass::Configure(const rapidjson::Value& Config,std::string RootPath) {
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

      ElementPtr ele = deftree.getElement(InputKeys_.back());
      if ( ele ) {
        config_.input_nodes.push_back(ele);
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
    data_.saturated_node = deftree.initElement(SaturatedKey_, "Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit", LOG_UINT8, LOG_NONE);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      config_.UpperLimit = Config["Limits"]["Upper"].GetFloat();
      config_.LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // pointer to log command data
  string path = Config["Output"].GetString();
  if ( path.size() && path[0] == '/' ) {
    // absolution path
    OutputKey_ = Config["Output"].GetString();
  } else {
    OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  }
  data_.output_node = deftree.initElement(OutputKey_, "Control law output", LOG_FLOAT, LOG_NONE);
}

void SumClass::Initialize() {}
bool SumClass::Initialized() {return true;}

void SumClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;

  float val = 0.0;
  for (size_t i=0; i < config_.input_nodes.size(); i++) {
    val += config_.input_nodes[i]->getFloat();
  }

  // saturate command
  if (config_.SaturateOutput) {
    if (val <= config_.LowerLimit) {
      val = config_.LowerLimit;
      data_.saturated_node->setInt(-1);
    } else if (val >= config_.UpperLimit) {
      val = config_.UpperLimit;
      data_.saturated_node->setInt(1);
    } else {
      data_.saturated_node->setInt(0);
    }
  }
  
  data_.output_node->setFloat(val);
}

void SumClass::Clear() {
  config_.input_nodes.clear();
  config_.LowerLimit = 0.0f;
  config_.UpperLimit = 0.0f;
  config_.SaturateOutput = false;
  data_.Mode = kStandby;
  data_.output_node->setFloat(0.0f);
  data_.saturated_node->setInt(0.0f);
  deftree.Erase(SaturatedKey_);
  deftree.Erase(OutputKey_);
  InputKeys_.clear();
  SaturatedKey_.clear();
  OutputKey_.clear();
}

/* Product class methods, see general-functions.hxx for more information */
void ProductClass::Configure(const rapidjson::Value& Config,std::string RootPath) {
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

      ElementPtr ele = deftree.getElement(InputKeys_.back());
      if ( ele ) {
        config_.input_nodes.push_back(ele);
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
    data_.saturated_node = deftree.initElement(SaturatedKey_, "Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit", LOG_UINT8, LOG_NONE);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      config_.UpperLimit = Config["Limits"]["Upper"].GetFloat();
      config_.LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  data_.output_node = deftree.initElement(OutputKey_, "Control law output", LOG_FLOAT, LOG_NONE);
}

void ProductClass::Initialize() {}
bool ProductClass::Initialized() {return true;}

void ProductClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;

  data_.output_node->setFloat(0.0f);
  float product = 1.0;
  for (size_t i=0; i < config_.input_nodes.size(); i++) {
    product *= config_.input_nodes[i]->getFloat();
  }
  data_.output_node->setFloat(product);

  // saturate command
  if (config_.SaturateOutput) {
    if (data_.output_node->getFloat() <= config_.LowerLimit) {
      data_.output_node->setFloat(config_.LowerLimit);
      data_.saturated_node->setInt(-1);
    } else if (data_.output_node->getFloat() >= config_.UpperLimit) {
      data_.output_node->setFloat(config_.UpperLimit);
      data_.saturated_node->setInt(1);
    } else {
      data_.saturated_node->setInt(0);
    }
  }
}

void ProductClass::Clear() {
  config_.input_nodes.clear();
  config_.LowerLimit = 0.0f;
  config_.UpperLimit = 0.0f;
  config_.SaturateOutput = false;
  data_.Mode = kStandby;
  data_.output_node->setFloat(0.0f);
  data_.saturated_node->setInt(0.0f);
  deftree.Erase(SaturatedKey_);
  deftree.Erase(OutputKey_);
  InputKeys_.clear();
  SaturatedKey_.clear();
  OutputKey_.clear();
}


/* Latch class methods, see general-functions.hxx for more information */
void LatchClass::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Input")) {
    InputKey_ = Config["Input"].GetString();
    config_.input_node = deftree.getElement(InputKey_);
    if ( !config_.input_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input ")+InputKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input not specified in configuration."));
  }

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  data_.output_node = deftree.initElement(OutputKey_, "Control law output", LOG_FLOAT, LOG_NONE);
}

void LatchClass::Initialize() {}
bool LatchClass::Initialized() {return true;}

void LatchClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;

  switch(data_.Mode) {
    case GenericFunction::Mode::kEngage: {
      if (initLatch_ == false) {
        initLatch_ = true;
        data_.output_node->setFloat(config_.input_node->getFloat());
      }
      break;
    }
    default: {
      initLatch_ = false;
    }
  }
}

void LatchClass::Clear() {
  data_.Mode = kStandby;
  data_.output_node->setFloat(0.0f);
  deftree.Erase(OutputKey_);
  InputKey_.clear();
  OutputKey_.clear();
}
