/*
effector.h
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

#ifndef EFFECTOR_H_
#define EFFECTOR_H_

#include "Arduino.h"
#include "../common/ArduinoJson.h"
#include "../common/SBUS.h"
#include "../common/Node.h"
#include "definition-tree.h"
#include "hardware-defs.h"
#include "utils.h"
#include <memory>

class AircraftEffectors {
  public:
    void UpdateConfig(const char *JsonString,DefinitionTree *DefinitionTreePtr);
    void Begin();
    void SetCommands(std::vector<float> Commands,bool ThrottleSafed);
    void ComputeOutputs(bool ThrottleSafed);
    void CommandEffectors();
    void End();
  private:
    SBUS *sbus_;
    enum EffectorType {
      kMotor,
      kPwm,
      kSbus
    };
    struct Config {
      float Period;
      float Resolution;
    };
    struct Data {
      EffectorType Type;
      float *Input;
      std::vector<float> Calibration;
      uint8_t Channel;
      float Output;
      float SafedCommand;
    };
    struct NodeData {
      Node *node;
      uint8_t Address;
      std::vector<float*> Inputs;
      std::vector<EffectorType> Types;
      std::vector<float> SafedCommands;
    };
    Config config_;
    std::vector<Data> Effectors_;
    std::vector<NodeData> NodeEffectors_;
    std::vector<float> EffectorCommands_;
};

#endif
