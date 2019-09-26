/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
*/

#pragma once

#include "rapidjson/document.h"
#include "definition-tree2.h"

#include "configuration.h"
#include "generic-function.h"

/*
MinCellVolt - Computes the minimum cell voltage amongst a number of batteries

Example JSON configuration:
{
  "Type": "MinCellVolt",
  "Output": "OutputName",
  "Inputs": ["InputPaths"],
  "NumCells": [X]
}
Where:
   * Inputs gives the full path of the inputs
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * NumCells is the number of cells in the battery

*/
class MinCellVolt: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string SystemName);
    void Initialize();
    bool Initialized();
    void Run(GenericFunction::Mode mode);
    void Clear();
  private:
    uint8_t mode = kStandby;
    vector<ElementPtr> input_nodes_;
    ElementPtr output_node_;

    std::vector<float> numCells_;
    std::vector<std::string> InputKeys_;
};
