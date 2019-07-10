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

void MinCellVolt::Configure(const rapidjson::Value& Config, std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Inputs", input_nodes_, &InputKeys_);
  LoadOutput(Config, SystemName, "Output", output_node_);

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
  GenericFunction::Mode Mode = kStandby;
}
