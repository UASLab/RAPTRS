/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
*/

#include "power.h"

void MinCellVolt::Configure(const rapidjson::Value& Config, std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Inputs", &input_nodes_, &InputKeys_);
  LoadOutput(Config, SystemName, "Output", &output_node_);

  // Number of cells
  LoadVal(Config, "NumCells", &numCells_, true);

}

void MinCellVolt::Initialize() {}
bool MinCellVolt::Initialized() {return true;}

void MinCellVolt::Run(GenericFunction::Mode mode) {
  float MinVoltPerCell;
  float VoltPerCell;

  // Compute the Minimum Voltage per Cell
  MinVoltPerCell = 4.3;
  for (size_t i=0; i < input_nodes_.size(); i++) {
    VoltPerCell = input_nodes_[i]->getFloat() / numCells_[i];

    if (VoltPerCell < MinVoltPerCell) {
      MinVoltPerCell = VoltPerCell;
    }
  }
  output_node_->setFloat(MinVoltPerCell);
}

void MinCellVolt::Clear() {
  // GenericFunction::Mode Mode = kStandby;
}
