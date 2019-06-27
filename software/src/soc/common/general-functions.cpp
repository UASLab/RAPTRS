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

/* Constant class methods, see general-functions.h for more information */
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
  data_.output_node->setFloat(config_.Constant);
}

void ConstantClass::Clear() {
  config_.Constant = 0.0f;
  data_.output_node->setFloat(0.0f);
  deftree.Erase(OutputKey_);
  OutputKey_.clear();
}

/* Gain class methods, see general-functions.h for more information */
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

  if (Config.HasMember("Min")) {
    config_.Min = Config["Min"].GetFloat();
  }
  if (Config.HasMember("Max")) {
    config_.Max = Config["Max"].GetFloat();
  }

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  data_.output_node = deftree.initElement(OutputKey_, "Control law output", LOG_FLOAT, LOG_NONE);
}

void GainClass::Initialize() {}
bool GainClass::Initialized() {return true;}

void GainClass::Run(Mode mode) {
  float val = config_.input_node->getFloat()*config_.Gain;

  // saturate command
  if (val <= config_.Min) {
    val = config_.Min;
  } else if (val >= config_.Max) {
    val = config_.Max;
  }

  data_.output_node->setFloat( val );
}

void GainClass::Clear() {
  config_.Gain = 1.0f;
  config_.Min = 0.0f;
  config_.Max = 0.0f;
  data_.output_node->setFloat(0.0f);
  deftree.Erase(OutputKey_);
  InputKey_.clear();
  OutputKey_.clear();
}

/* Sum class methods, see general-functions.h for more information */
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


  if (Config.HasMember("Min")) {
    config_.Min = Config["Min"].GetFloat();
  }
  if (Config.HasMember("Max")) {
    config_.Max = Config["Max"].GetFloat();
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
  float sum = 0.0;
  for (size_t i=0; i < config_.input_nodes.size(); i++) {
    sum += config_.input_nodes[i]->getFloat();
  }

  // saturate command
  if (sum <= config_.Min) {
    sum = config_.Min;
  } else if (sum >= config_.Max) {
    sum = config_.Max;
  }

  data_.output_node->setFloat(sum);
}

void SumClass::Clear() {
  config_.input_nodes.clear();
  config_.Min = 0.0f;
  config_.Max = 0.0f;
  data_.output_node->setFloat(0.0f);
  deftree.Erase(OutputKey_);
  InputKeys_.clear();
  OutputKey_.clear();
}

/* Product class methods, see general-functions.h for more information */
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

  if (Config.HasMember("Min")) {
    config_.Min = Config["Min"].GetFloat();
  }
  if (Config.HasMember("Max")) {
    config_.Max = Config["Max"].GetFloat();
  }

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  data_.output_node = deftree.initElement(OutputKey_, "Control law output", LOG_FLOAT, LOG_NONE);
}

void ProductClass::Initialize() {}
bool ProductClass::Initialized() {return true;}

void ProductClass::Run(Mode mode) {
  float product = 1.0;
  for (size_t i=0; i < config_.input_nodes.size(); i++) {
    product *= config_.input_nodes[i]->getFloat();
  }

  // saturate command
  if (product <= config_.Min) {
    product = config_.Min;
  } else if (product >= config_.Max) {
    product = config_.Max;
  }

  data_.output_node->setFloat(product);
}

void ProductClass::Clear() {
  config_.input_nodes.clear();
  config_.Min = 0.0f;
  config_.Max = 0.0f;
  data_.output_node->setFloat(0.0f);
  deftree.Erase(OutputKey_);
  InputKeys_.clear();
  OutputKey_.clear();
}


/* Delay class methods, see general-functions.h for more information */
void DelayClass::Configure(const rapidjson::Value& Config, std::string RootPath) {
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

  if (Config.HasMember("Delay_frames")) {
    config_.delay_frames = Config["Delay_frames"].GetInt();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Delay_frames not specified in configuration."));
  }

  // pointer to log command data
  OutputKey_ = RootPath+"/"+Config["Output"].GetString();
  data_.output_node = deftree.initElement(OutputKey_, "Control law (Delay) output", LOG_FLOAT, LOG_NONE);
}

void DelayClass::Initialize() {
  data_.buffer.clear();
}

bool DelayClass::Initialized() { return true; }

void DelayClass::Run(Mode mode) {
  data_.buffer.push_back( config_.input_node->getFloat() );
  float val = 0.0;
  while ( data_.buffer.size() > 0 && data_.buffer.size() > config_.delay_frames ) {
    val = data_.buffer.front();
    data_.buffer.pop_front();
  }
  data_.output_node->setFloat(val);
}

void DelayClass::Clear() {
  config_.delay_frames = 0;
  data_.buffer.clear();
  data_.output_node->setFloat(0.0f);
  deftree.Erase(OutputKey_);
  InputKey_.clear();
  OutputKey_.clear();
}


/* Latch class methods, see general-functions.h for more information */
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
  switch(mode) {
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
  data_.output_node->setFloat(0.0f);
  deftree.Erase(OutputKey_);
  InputKey_.clear();
  OutputKey_.clear();
}
