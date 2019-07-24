/*
configuration.hxx
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

#pragma once

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree2.h"

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <stdint.h>

class Configuration {
  public:
    void LoadConfiguration(std::string FileName,rapidjson::Document *Configuration);
};

// Configuration Helpers: Read Json, Configure definition tree
std::string ParseInput(std::string SystemPath, std::string Key);
void LoadInput(const rapidjson::Value& Config, std::string SystemName, std::string InputName, ElementPtr *Node, std::string *InputKey);
void LoadInput(const rapidjson::Value& Config, std::string SystemName, std::string InputName, std::vector<ElementPtr> *Node, std::vector<std::string> *InputKey);

void LoadOutput(const rapidjson::Value& Config, std::string SystemName, std::string OutputName, ElementPtr *Node);
void LoadOutput(const rapidjson::Value& Config, std::string SystemName, std::string OutputName, std::vector<ElementPtr> *Node);

void LoadVal(const rapidjson::Value& Config, std::string ValName, int *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, float *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, std::string *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string VecName, std::vector<float> *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, std::vector<std::string> *Val, bool required);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::ArrayXf *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::VectorXf *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string MatName, std::vector<std::vector<float>> *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::MatrixXf *Val, bool required = 0);
