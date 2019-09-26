/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
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
