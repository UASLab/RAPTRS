/*
power.cc

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

#include "power.h"

void MinCellVolt::Configure(const rapidjson::Value& Config,std::string RootPath) {
  // grab inputs
  if (Config.HasMember("Inputs")) {
    for (size_t i=0; i < Config["Inputs"].Size(); i++) {
      const rapidjson::Value& Input = Config["Inputs"][i];
      InputKeys_.push_back(Input.GetString());
      Element *ele = deftree.getElement(InputKeys_.back());
      if ( ele ) {
        config_.input_nodes.push_back(ele);
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Input ")+InputKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR") + RootPath + std::string(": Inputs not specified in configuration."));
  }

  // get output name
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
    data_.output_node = deftree.initElement(OutputName, "Min Cell Voltage output", LOG_FLOAT, LOG_NONE);
  } else {
    throw std::runtime_error(std::string("ERROR") + RootPath + std::string(": Output not specified in configuration."));
  }

  // grab number of cells
  if (Config.HasMember("NumCells")) {
    for (size_t i=0; i < Config["NumCells"].Size(); i++) {
      numCells.push_back(Config["NumCells"][i].GetFloat());
    }
  } else {
    throw std::runtime_error(std::string("ERROR") + OutputName + std::string(": NumCells not specified in configuration."));
  }
}

void MinCellVolt::Initialize() {}
bool MinCellVolt::Initialized() {return true;}

void MinCellVolt::Run(Mode mode) {
  float MinVoltPerCell;
  float VoltPerCell;

  // Compute the Minimum Voltage per Cell
  MinVoltPerCell = 4.3;
  for (size_t i=0; i < config_.input_nodes.size(); i++) {
    VoltPerCell = config_.input_nodes[i]->getFloat() / numCells[i];

    if (VoltPerCell < MinVoltPerCell) {
      MinVoltPerCell = VoltPerCell;
    }
  }
  data_.output_node->setFloat(MinVoltPerCell);
}

void MinCellVolt::Clear() {
  data_.Mode = (uint8_t) kStandby;
}
