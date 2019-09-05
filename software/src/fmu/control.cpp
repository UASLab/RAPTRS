/*
control.cc
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

#include "control.h"
#include "utils.h"

void ControlFunctionClass::Configure(message::config_control_gain_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {}
void ControlFunctionClass::Run(Mode mode) {}

/* control gain class methods */

void ControlGainClass::Configure(message::config_control_gain_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
    std::string OutputName;
    OutputName = RootPath + "/" + msg->output;
    config_.Gain = msg->gain;
    if (DefinitionTreePtr->GetValuePtr<float*>(msg->input.c_str())) {
      config_.Input = DefinitionTreePtr->GetValuePtr<float*>(msg->input.c_str());
    } else {
      HardFail("ERROR: Control Input not found in global data.");
    }
    if ( msg->has_limits ) {
      config_.SaturateOutput = true;
      config_.UpperLimit = msg->upper_limit;
      config_.LowerLimit = msg->lower_limit;
    }
    // pointer to log run mode data
    // DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode);
    // pointer to log command data
    DefinitionTreePtr->InitMember(OutputName, &data_.Command);
}

/* gain block run method, outputs the mode and value */
void ControlGainClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  switch (mode) {
    // Zero the State and Command
    case kReset: {
      data_.Command = 0;
      data_.Saturated = 0;
      break;
    }
    // Do Nothing, State and Command are unchanged
    case kStandby: {
      break;
    }
    // Run Commands
    case kHold: {
      CalculateCommand();
      break;
    }
    // Initialize State then Run Commands
    case kInitialize: {
      CalculateCommand();
      break;
    }
    // Update the State then Run Commands
    case kEngage: {
      CalculateCommand();
      break;
    }
  }
}

/* calculate the command and apply saturation, if enabled */
void ControlGainClass::CalculateCommand() {
  // Serial.print(*config_.Input); Serial.print(" "); Serial.print(config_.Gain);
  data_.Command = *config_.Input*config_.Gain;
  // Serial.print(" = "); Serial.println(data_.Command);
  // saturate command
  if (config_.SaturateOutput) {
    if (data_.Command <= config_.LowerLimit) {
      data_.Command = config_.LowerLimit;
      data_.Saturated = -1;
    } else if (data_.Command >= config_.UpperLimit) {
      data_.Command = config_.UpperLimit;
      data_.Saturated = 1;
    } else {
      data_.Saturated = 0;
    }
  }
}

/* configures control laws and registers data with global defs */
bool ControlLaws::Configure(uint8_t id, std::vector<uint8_t> *Payload, DefinitionTree *DefinitionTreePtr) {
  if ( id != message::config_control_gain_id ) {
    // not our message
    return false;
  }
  message::config_control_gain_t msg;
  msg.unpack(Payload->data(), Payload->size());
  // configuring our path
  std::string PathName = RootPath_;
  // resizing the control group for the new gain
  size_t i = BaselineControlGroup_.size() + 1;
  BaselineControlGroup_.resize(i + 1);
  ControlGainClass Temp;
  size_t j = BaselineControlGroup_[i].size();
  BaselineControlGroup_[i].push_back(std::make_shared<ControlGainClass>(Temp));
  BaselineControlGroup_[i][j]->Configure(&msg, PathName, DefinitionTreePtr);
}

/* returns the number of levels for the engaged control law */
size_t ControlLaws::ActiveControlLevels() {
  return BaselineControlGroup_.size();
}

/* computes control law data */
void ControlLaws::Run(size_t ControlLevel) {
  for (size_t i=0; i < BaselineControlGroup_[ControlLevel].size(); i++) {
    // Serial.print(i); Serial.print(": ");
    BaselineControlGroup_[ControlLevel][i]->Run(ControlFunctionClass::kEngage);
  }
}

/* free resources used by control laws */
void ControlLaws::End() {
  BaselineControlGroup_.clear();
}
