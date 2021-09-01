/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "Arduino.h"
#include "definition-tree.h"
#include <memory>

#include "fmu_messages.h"

/* base function class methods */

class ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    virtual void Configure(message::config_control_gain_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    virtual void Run(Mode mode);
};

class ControlGainClass: public ControlFunctionClass {
  public:
    void Configure(message::config_control_gain_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
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
    bool Configure(uint8_t id, std::vector<uint8_t> *Payload, DefinitionTree *DefinitionTreePtr);
    bool Configured();
    size_t ActiveControlLevels();
    void Run(size_t ControlLevel);
    void End();
  private:
    std::string RootPath_ = "/Control";
    std::vector<std::vector<std::shared_ptr<ControlFunctionClass>>> BaselineControlGroup_;
};

