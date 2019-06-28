/*
configuration.cxx
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

#include "configuration.h"

void Configuration::LoadConfiguration(std::string FileName,rapidjson::Document *Configuration) {
  // load config file
  std::ifstream ConfigFile(FileName);
  std::string ConfigBuffer((std::istreambuf_iterator<char>(ConfigFile)),std::istreambuf_iterator<char>());
  // parse JSON
  rapidjson::StringStream JsonConfig(ConfigBuffer.c_str());
  Configuration->ParseStream(JsonConfig);
  assert(Configuration->IsObject());
}


// Helpers to Read Config
std::string Configuration::LoadOutput(const rapidjson::Value& Config, std::string SystemName, std::string OutputName, ElementPtr Node) {
  if (Config.HasMember(OutputName)) {
    std::string OutputKey = Config[OutputName].GetString();
    Node = deftree.initElement(SystemName + "/" + OutputName, ": System output", LOG_FLOAT, LOG_NONE);
  } else {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + OutputName + std::string(" not specified in configuration."));
  }
  return OutputKey;
}

std::string Configuration::LoadInput(const rapidjson::Value& Config, std::string SystemName, std::string InputName, ElementPtr Node) {
  if (Config.HasMember(InputName)) {
    std::string InputKey = Config[InputName].GetString();
    Node = deftree.getElement(InputKey, true);
  } else {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + InputName + std::string(" not specified in configuration."));
  }
  return InputKey;
}

std::string Configuration::LoadTime(const rapidjson::Value& Config, std::string SystemName, std::string TimeName, ElementPtr Node) {
  if (Config.HasMember(TimeName)) {
    std::string TimeKey = Config[TimeName].GetString();
    Node = deftree.getElement(TimeKey);
  } else {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + TimeName + std::string(" not specified in configuration."));
  }
  return TimeKey;
}

float Configuration::LoadValue(const rapidjson::Value& Config, std::string ValName) {
  float Val;
  if (Config.HasMember(ValName)) {
    Val = Config[ValName].GetFloat();
  }
  return Val;
}
