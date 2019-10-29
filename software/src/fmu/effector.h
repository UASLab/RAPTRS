/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#ifndef EFFECTOR_H_
#define EFFECTOR_H_

#include "Arduino.h"
#include "SBUS.h"
#include "definition-tree.h"
#include "hardware-defs.h"
#include "utils.h"
#include <memory>

#include "Node.h"

#include "fmu_messages.h"

class AircraftEffectors {
  public:
    bool UpdateConfig(uint8_t id, uint8_t address, std::vector<uint8_t> *Payload, DefinitionTree *DefinitionTreePtr);
    void Begin();
    void SetCommands(message::command_effectors_t *msg, bool ThrottleSafed);
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
    //std::vector<float> EffectorCommands_;
};

#endif
