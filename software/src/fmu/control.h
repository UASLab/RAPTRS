/*
control.hxx
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

#ifndef CONTROL_HXX_
#define CONTROL_HXX_

#include "Arduino.h"
#include "definition-tree.h"
#include "ArduinoJson.h"
#include <memory>

class ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    virtual void Configure(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    virtual void Run(Mode mode);
};

class ControlGainClass: public ControlFunctionClass {
  public:
    void Configure(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Run(Mode mode);
  private:
    struct Config {
      float *Input;
      float Gain = 1;
      bool SaturateOutput = false;
      float UpperLimit, LowerLimit = 0;
    };
    struct Data {
      uint8_t Mode = kStandby;
      float Command = 0;
      int8_t Saturated = 0;
    };
    Config config_;
    Data data_;
    void CalculateCommand();
};

class ControlLaws {
  public:
    void Configure(const char *JsonString,DefinitionTree *DefinitionTreePtr);
    bool Configured();
    size_t ActiveControlLevels();
    void Run(size_t ControlLevel);
    void End();
  private:
    std::string RootPath_ = "/Control";
    std::vector<std::vector<std::shared_ptr<ControlFunctionClass>>> BaselineControlGroup_;
};

#endif
