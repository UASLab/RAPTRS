/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
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
void LoadVal(const rapidjson::Value& Config, std::string ValName, uint *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, float *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, std::string *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string VecName, std::vector<float> *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, std::vector<std::string> *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::ArrayXf *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::Vector3f *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::VectorXf *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::Vector3d *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::VectorXd *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string MatName, std::vector<std::vector<float>> *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string ValName, Eigen::MatrixXf *Val, bool required = 0);
void LoadVal(const rapidjson::Value& Config, std::string MatName, std::vector<Eigen::Vector3f> *Val, bool required = 0);
