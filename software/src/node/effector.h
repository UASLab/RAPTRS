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
#include "SBUS.h"
#include "hardware-defs.h"
#include "utils.h"
#include <memory>

#include "fmu_messages.h"

class AircraftEffectors {
  public:
    bool UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload);
    void SetCommands(std::vector<float> Commands);
    void ComputeOutputs();
    void CommandEffectors();
    void End();
  private:
    SBUS *sbus_ = NULL;
    enum EffectorType {
      kPwm,
      kSbus
    };
    struct Config {
      float Period;
      float Resolution;
    };
    struct Data {
      EffectorType Type;
      std::vector<float> Calibration;
      uint8_t Channel;
      float Output;
    };
    Config config_;
    std::vector<Data> Effectors_;
    std::vector<float> EffectorCommands_;
};

#endif
