/*
allocation-functions.hxx
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

#ifndef ALLOCATION_FUNCTIONS_HXX_
#define ALLOCATION_FUNCTIONS_HXX_

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree2.h"
#include "generic-function.h"

#include <Eigen/Dense>
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
   * Input gives the full path of the allocator inputs / objectives (i.e. /Control/PitchMomentCmd)
   * Output gives the relative path of the allocator outputs / effector commands (i.e Elevator)
   * Effectiveness gives the control effectiveness (i.e. change in moment for a unit change in effector output)
     The order is MxN where M is the number of outputs / effectors and N is the number of inputs / objectives. So,
     for a situation with 3 objectives (i.e. pitch, roll, yaw moments) and 7 control surfaces, Effectiveness would
     be given as:
     "Effectiveness":[[PitchEff_Surf0,RollEff_Surf0,YawEff_Surf0],
                      [PitchEff_Surf1,RollEff_Surf1,YawEff_Surf1],
                      .
                      .
                      .
                      [PitchEff_Surf6,RollEff_Surf6,YawEff_Surf6]]
   * Limits gives the upper and lower limits for each output / effector command.
*/

class PseudoInverseAllocation: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      vector<ElementPtr> input_nodes;
      Eigen::VectorXf Objectives;
      Eigen::MatrixXf Effectiveness;
      Eigen::VectorXf Min;
      Eigen::VectorXf Max;
    };
    struct Data {
      Eigen::VectorXf uCmd;
      vector<ElementPtr> uCmd_nodes;
    };
    int numIn, numOut;
    Config config_;
    Data data_;
    std::vector<std::string> InputKeys_;
};

#endif
