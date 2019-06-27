/*
filter-functions.hxx
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

#ifndef FILTER_FUNCTIONS_HXX_
#define FILTER_FUNCTIONS_HXX_

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree2.h"
#include "generic-function.h"
#include "filter-algorithms.h"

/*
General Filter - Implements a general discrete time filter using the
general filter difference equation.

den[0]y[n] = num[0]x[n]+num[1]x[n-1]+num[2]x[n-2]+...-den[1]y[n-1]-den[2]y[n-2]-...

Example JSON configuration:
{
  "Type": "Filter",
  "Input": "InputPath",
  "Output": "OutputName",
  "num": [X],
  "den": [X]
}
Where:
   * Input gives the full path of the input to the filter
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * den is a vector of denominator coefficients. den[0] scales all den and num coefficients if given.
     Denominator coefficients are optional and, if none are provided, a FIR filter is implemented.
   * num is a vector of numerator coefficients. At least one feedforward coefficient must be given.
The order of the filter is given by the length of the num and den vectors.
*/
class GeneralFilter: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      ElementPtr input_node;
    };
    struct Data {
      ElementPtr output_node;
    };
    __GeneralFilter filter_;
    Config config_;
    Data data_;
    std::string InputKey_,OutputKey_;
};

#endif
