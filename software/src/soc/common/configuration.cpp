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
// Load an Config output and create a node in the deftree
void LoadOutput(const rapidjson::Value& Config, std::string SystemName, std::string OutputName, ElementPtr Node) {

  if (Config.HasMember(OutputName.c_str())) {
    Node = deftree.initElement(SystemName + "/" + OutputName, "System output", LOG_FLOAT, LOG_NONE);
  } else {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + OutputName + std::string(" not specified in configuration."));
  }
}

void LoadOutput(const rapidjson::Value& Config, std::string SystemName, std::string OutputName, std::vector<ElementPtr> Node) {
  if (Config.HasMember(OutputName.c_str())) {
    Node.resize(Config[OutputName.c_str()].Size());

    for (size_t i=0; i < Config[OutputName.c_str()].Size(); i++) {
      const rapidjson::Value& Elem = Config[OutputName.c_str()][i];
      std::string ElemName = Elem.GetString();

      // pointer to log output
      Node[i] = deftree.initElement(SystemName + "/" + ElemName, "System output", LOG_FLOAT, LOG_NONE);
    }
  } else {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + OutputName + std::string(" not specified in configuration."));
  }
}

// Load an Config input to an existing node in the deftree
void LoadInput(const rapidjson::Value& Config, std::string SystemName, std::string InputName, ElementPtr Node, std::string *InputKey) {
  if (Config.HasMember(InputName.c_str())) {
    (*InputKey) = Config[InputName.c_str()].GetString();
    Node = deftree.getElement(*InputKey, true);
  } else {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + InputName + std::string(" not specified in configuration."));
  }
}

void LoadInput(const rapidjson::Value& Config, std::string SystemName, std::string InputName, vector<ElementPtr> Node, vector<std::string> *InputKey) {
  if (Config.HasMember(InputName.c_str())) {
    for (size_t i=0; i < Config[InputName.c_str()].Size(); i++) {
      const rapidjson::Value& Input = Config[InputName.c_str()][i];
      (*InputKey).push_back(Input.GetString());

      Node.push_back(deftree.getElement((*InputKey).back(), true));
    }
  } else {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + InputName + std::string(": not specified in configuration."));
  }
}


// Get a simple value, overloaded
void LoadVal(const rapidjson::Value& Config, std::string ValName, int *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    (*Val) = Config[ValName.c_str()].GetInt();
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}
void LoadVal(const rapidjson::Value& Config, std::string ValName, float *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    (*Val) = Config[ValName.c_str()].GetFloat();
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}

void LoadVal(const rapidjson::Value& Config, std::string ValName, std::string *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    (*Val) = Config[ValName.c_str()].GetString();
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}

void LoadVal(const rapidjson::Value& Config, std::string ValName, std::vector<float> *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      (*Val).push_back(Config[ValName.c_str()][m].GetFloat());
    }
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}

void LoadVal(const rapidjson::Value& Config, std::string ValName, std::vector<std::string> *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      (*Val).push_back(Config[ValName.c_str()][m].GetString());
    }
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}

void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::ArrayXf *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    (*Val).resize(Config[ValName.c_str()].Size());

    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      (*Val)(m) = Config[ValName.c_str()][m].GetFloat();
    }
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}

void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::VectorXf *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    (*Val).resize(Config[ValName.c_str()].Size());

    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      (*Val)(m) = Config[ValName.c_str()][m].GetFloat();
    }
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}

void LoadVal(const rapidjson::Value& Config, std::string ValName, std::vector<std::vector<float>> *Val, bool required) {
  std::vector<float> ValVec;
  if (Config.HasMember(ValName.c_str())) {
    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      ValVec.clear();
      for (size_t n=0; n < Config[ValName.c_str()][m].Size(); n++) {
        ValVec.push_back(Config[ValName.c_str()][m].GetFloat());
      }
      (*Val).push_back(ValVec);
    }
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}

void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::MatrixXf *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    (*Val).resize(Config[ValName.c_str()].Size(),Config[ValName.c_str()][0].Size());

    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      for (size_t n=0; n < Config[ValName.c_str()][m].Size(); n++) {
        (*Val)(m,n) = Config[ValName.c_str()][m][n].GetFloat();
      }
    }
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}
