/*
flow-control-functions.hxx
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

#ifndef FLOW_CONTROL_FUNCTIONS_HXX_
#define FLOW_CONTROL_FUNCTIONS_HXX_

#include "definition-tree.hxx"
#include "generic-function.hxx"
/* 
If Class - Outputs a 0 or 1 depending on input and threshold values
Example JSON configuration:
{
  "Type": "If",
  "Output": "OutputName",
  "Input": "InputName",
  "Threshold": X
}
Where: 
   * Output gives a convenient name for the block (i.e. AmFlying).
   * Input is the full path of the input value
   * Threshold is a float threshold value
Data type for the output is uint8_t, 1 if the input value is greater than the threshold, otherwise 0
*/
class If: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      float *Input;
      float Threshold = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;
      uint8_t Output = 0;
    };
    Config config_;
    Data data_;
    std::string InputKey_,ModeKey_,OutputKey_;
};

#endif
