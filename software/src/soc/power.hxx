/*
power.hxx

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

#ifndef POWER_HXX_
#define POWER_HXX_

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree.hxx"
#include "generic-function.hxx"

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
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      std::vector<float*> Inputs;
    };
    struct Data {
      uint8_t Mode = kStandby;
      float Output = 0.0f;
    };

    std::vector<float> numCells;

    Config config_;
    Data data_;

    std::vector<std::string> InputKeys_;
};

#endif // POWER_HXX_
