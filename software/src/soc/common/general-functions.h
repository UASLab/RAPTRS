/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "generic-function.h"
#include "configuration.h"

#include <limits>
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
    float Val_ = 0.0f;
    ElementPtr Output_node_;
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
  "Min": X, "Max": X
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
    float Gain_ = 1.0f;
    float Min_ = std::numeric_limits<float>::lowest();
    float Max_ = std::numeric_limits<float>::max();

    ElementPtr Input_node_;
    ElementPtr Output_node_;

    std::string InputKey_, OutputKey_;
};

/*
Sum Class - Sums all inputs
Example JSON configuration:
{
  "Type": "Sum",
  "Output": "OutputName",
  "Inputs": ["InputName1","InputName2",...],
  "Min": X, "Max": X
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
    float Min_ = std::numeric_limits<float>::lowest();
    float Max_ = std::numeric_limits<float>::max();

    std::vector<float > Gain_;

    std::vector<ElementPtr > Input_nodes_;
    ElementPtr Output_node_;

    std::vector<std::string> InputKeys_;
    std::string OutputKey_;
};

/*
Product Class - Multiplies all inputs
Example JSON configuration:
{
  "Type": "Product",
  "Output": "OutputName",
  "Inputs": ["InputName1","InputName2",...],
  "Min": X, "Max": X
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
    float Min_ = std::numeric_limits<float>::lowest();
    float Max_ = std::numeric_limits<float>::max();

    std::vector<ElementPtr > Input_nodes_;
    ElementPtr Output_node_;

    std::vector<std::string> InputKeys_;
    std::string OutputKey_;
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
    int DelayFrames_ = 0;
    deque<float> DelayBuffer_;

    ElementPtr Input_node_;
    ElementPtr Output_node_;

    std::string InputKey_,OutputKey_;
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
    ElementPtr Input_node_;
    ElementPtr Output_node_;

    bool InitLatch_ = false;

    std::string InputKey_,OutputKey_;
};

/*
ABS Class - Absolute Value
Example JSON configuration:
{
  "Type": "ABS",
  "Output": "OutputName",
  "Input": "InputName",
  }
}
Where:
   * Output gives a convenient name for the block (i.e. SpeedControl).
   * Input is the full path name of the input signal.
Data types for the input and output are both float.
*/
class ABSClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    ElementPtr Input_node_;
    ElementPtr Output_node_;

    std::string InputKey_;
};
