/*
general-functions.hxx
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

#ifndef GENERAL_FUNCTIONS_HXX_
#define GENERAL_FUNCTIONS_HXX_

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree2.h"
#include "generic-function.h"

#include <deque>
using std::deque;

/*
Constant Class - Outputs a constant value.
Example JSON configuration:
{
  "Type": "Constant",
  "Output": "OutputName",
  "Constant": X
}
Where:
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * Constant is the value of the constant output.
Data type for the output is float.
*/
class ConstantClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      float Constant = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;
      ElementPtr output_node;
    };
    Config config_;
    Data data_;
    std::string OutputKey_;
};

/*
Gain Class - Multiplies an input by a gain
Example JSON configuration:
{
  "Type": "Gain",
  "Output": "OutputName",
  "Input": "InputName",
  "Gain": X,
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where:
   * Output gives a convenient name for the block (i.e. SpeedControl).
   * Input is the full path name of the input signal.
   * Gain is the gain applied to the input signal.
   * Limits are optional and saturate the output if defined.
Data types for the input and output are both float.
*/
class GainClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      ElementPtr input_node;
      float Gain = 1.0f;
      bool SaturateOutput = false;
      float UpperLimit, LowerLimit = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;
      ElementPtr output_node;
      ElementPtr saturated_node;
    };
    Config config_;
    Data data_;
    std::string InputKey_,SaturatedKey_,OutputKey_;
};

/*
Sum Class - Sums all inputs
Example JSON configuration:
{
  "Type": "Sum",
  "Output": "OutputName",
  "Inputs": ["InputName1","InputName2",...],
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where:
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * Inputs is a vector of full path names of the input signals. All inputs
     will be summed.
   * Limits are optional and saturate the output if defined.
Data types for the input and output are both float.
*/
class SumClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      std::vector<ElementPtr > input_nodes;
      bool SaturateOutput = false;
      float UpperLimit, LowerLimit = 0.0f;
      bool debug = false;
    };
    struct Data {
      uint8_t Mode = kStandby;
      ElementPtr output_node;
      ElementPtr saturated_node;
    };
    Config config_;
    Data data_;
    std::vector<std::string> InputKeys_;
    std::string SaturatedKey_,OutputKey_;
};

/*
Product Class - Multiplies all inputs
Example JSON configuration:
{
  "Type": "Product",
  "Output": "OutputName",
  "Inputs": ["InputName1","InputName2",...],
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where:
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * Inputs is a vector of full path names of the input signals. All inputs
     will be multiplied.
   * Limits are optional and saturate the output if defined.
Data types for the input and output are both float.
*/
class ProductClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      std::vector<ElementPtr > input_nodes;
      bool SaturateOutput = false;
      float UpperLimit, LowerLimit = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;
      ElementPtr output_node;
      ElementPtr saturated_node;
    };
    Config config_;
    Data data_;
    std::vector<std::string> InputKeys_;
    std::string SaturatedKey_,OutputKey_;
};

/*
Delay Class - Delay a signal by 'n' frames
Example JSON configuration:
{
  "Type": "Delay",
  "Output": "OutputName",
  "Input": "InputName",
  "Delay_frames": n
}
Where:
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * Input is the full path name of the input signal.
   * Output is the input signal delayed by n frames
Data types for the input and output are both float.
*/
class DelayClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      ElementPtr input_node;
      int delay_frames = 0;
    };
    struct Data {
      uint8_t Mode = kStandby;
      ElementPtr output_node;
      deque<float> buffer;
    };
    Config config_;
    Data data_;
    std::string InputKey_;
    std::string OutputKey_;
};

/*
Latch Class - Latches the Output to the initial input
Example JSON configuration:
{
  "Type": "Latch",
  "Output": "OutputName",
  "Input": "InputName",
  }
}
Where:
   * Output gives a convenient name for the block (i.e. SpeedControl).
   * Input is the full path name of the input signal.
Data types for the input and output are both float.
*/
class LatchClass: public GenericFunction {
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
      uint8_t Mode = kStandby;
      ElementPtr output_node;
    };
    bool initLatch_ = false;
    Config config_;
    Data data_;
    std::string InputKey_,OutputKey_;
};

#endif
