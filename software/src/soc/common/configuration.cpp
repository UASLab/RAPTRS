/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
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
void LoadOutput(const rapidjson::Value& Config, std::string SystemName, std::string OutputName, ElementPtr *Node) {
  if (!Config.HasMember(OutputName.c_str())) {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + OutputName + std::string(" not specified in configuration."));
  }

  const rapidjson::Value& Elem = Config[OutputName.c_str()];
  std::string OutKey = Elem.GetString();
  *Node = deftree.initElement(SystemName + "/" + OutKey, "System output", LOG_FLOAT, LOG_NONE);
}
void LoadOutput(const rapidjson::Value& Config, std::string SystemName, std::string OutputName, std::vector<ElementPtr> *Node) {
  if (!Config.HasMember(OutputName.c_str())) {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + OutputName + std::string(" not specified in configuration."));
  }

  (*Node).resize(Config[OutputName.c_str()].Size());

  for (size_t i=0; i < Config[OutputName.c_str()].Size(); i++) {
    const rapidjson::Value& Elem = Config[OutputName.c_str()][i];
    std::string ElemName = Elem.GetString();

    // pointer to log output
    (*Node)[i] = deftree.initElement(SystemName + "/" + ElemName, "System output", LOG_FLOAT, LOG_NONE);
  }

}

// Load an Config input to an existing node in the deftree
std::string ParseInput(std::string SystemPath, std::string Key) {
  // If key starts with "/" then it references a full Path (default, not checked)
  // If key starts with "../" then it references the next higher level
  // If key has no "/" path indicators then it references the current system
  if (Key.find("../") != std::string::npos) { // "../" found
    Key = Key.substr (Key.rfind("/") + 1);
    SystemPath = SystemPath.substr (0, SystemPath.rfind("/"));

    Key = SystemPath + "/" + Key;
  } else if (Key.find("/") == std::string::npos) { // "/" not found
    Key = Key.substr (Key.rfind("/")+1);

    Key = SystemPath + "/" + Key;
  }
  return Key;
}

// Get inputs, overloaded
void LoadInput(const rapidjson::Value& Config, std::string SystemName, std::string InputName, ElementPtr *Node, std::string *InputKey) {
  if (!Config.HasMember(InputName.c_str())) {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + InputName + std::string(" not specified in configuration."));
  }

  *InputKey = ParseInput(SystemName, Config[InputName.c_str()].GetString());
  *Node = deftree.getElement(*InputKey, true);
}
void LoadInput(const rapidjson::Value& Config, std::string SystemName, std::string InputName, vector<ElementPtr> *Node, vector<std::string> *InputKey) {
  if (!Config.HasMember(InputName.c_str())) {
    throw std::runtime_error(std::string("ERROR - ") + SystemName + std::string(" : ") + InputName + std::string(": not specified in configuration."));
  }

  for (size_t i=0; i < Config[InputName.c_str()].Size(); i++) {
    const rapidjson::Value& Input = Config[InputName.c_str()][i];
    (*InputKey).push_back(ParseInput(SystemName, Input.GetString()));
    (*Node).push_back(deftree.getElement((*InputKey).back(), true));
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
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::Vector3f *Val, bool required) {
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
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::Vector3d *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    (*Val).resize(Config[ValName.c_str()].Size());

    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      (*Val)(m) = Config[ValName.c_str()][m].GetDouble();
    }
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::VectorXd *Val, bool required) {
  if (Config.HasMember(ValName.c_str())) {
    (*Val).resize(Config[ValName.c_str()].Size());

    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      (*Val)(m) = Config[ValName.c_str()][m].GetDouble();
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
void LoadVal(const rapidjson::Value& Config, std::string ValName, std::vector<Eigen::Vector3f> *Val, bool required) {
  Eigen::Vector3f ValVec;
  if (Config.HasMember(ValName.c_str())) {
    for (size_t m=0; m < Config[ValName.c_str()].Size(); m++) {
      ValVec.resize(Config[ValName.c_str()][m].Size());

      for (size_t n=0; n < Config[ValName.c_str()][m].Size(); m++) {
        ValVec(m) = Config[ValName.c_str()][m].GetFloat();
      }

      (*Val).push_back(ValVec);
    }
  } else if (required) {
    throw std::runtime_error(std::string("ERROR - ") + ValName + std::string(" not specified in configuration."));
  }
}
