/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#pragma once

#include "configuration.h"
#include "generic-function.h"

#include <limits>

/*
Pseudo Inverse Allocation - Implements a pseudo inverse control allocation

Example JSON configuration:
{
  "Type": "PseudoInverse",
  "Inputs": [N],
  "Outputs": [M],
  "Effectiveness": [[N],[N],...],
  "Min": [M],
  "Max": [M]
}
Where:
   * Input gives the path of the allocator inputs / objectives (i.e. ../PitchMomentCmd)
   * Output gives the relative path of the allocator outputs / effector commands (i.e Elevator)
   * Effectiveness gives the control effectiveness (i.e. change in moment for a unit change in effector output)
     The order is NxM where M is the number of outputs / effectors and N is the number of inputs / objectives. So,
     for a situation with 3 objectives (i.e. pitch, roll, yaw moments) and 4 control surfaces, Effectiveness would
     be given as:
     "Effectiveness":[[RollEff_Surf0, RollEff_Surf1, RollEff_Surf2, RollEff_Surf3],
                      [PitchEff_Surf0, PitchEff_Surf1, PitchEff_Surf2, PitchEff_Surf3],
                      [YawEff_Surf0, YawEff_Surf1, YawEff_Surf2, YawEff_Surf3],]
   * Min and Max gives the upper and lower limits for each output / effector command.
*/

class PseudoInverseAllocation: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    Eigen::VectorXf Objectives;
    Eigen::MatrixXf Effectiveness;
    Eigen::VectorXf uCmd;

    Eigen::VectorXf Min;
    Eigen::VectorXf Max;

    vector<ElementPtr> input_nodes;
    vector<ElementPtr> uCmd_nodes;

    int numIn, numOut;

    std::vector<std::string> InputKeys_;
};
