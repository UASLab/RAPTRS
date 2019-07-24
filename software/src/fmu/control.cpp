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

/* base function class methods */
void ControlFunctionClass::Configure(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void ControlFunctionClass::Run(Mode mode) {}

/* control gain class methods */

/* method for configuring the gain block */
/* example JSON configuration:
{
  "Output": "OutputName",
  "Input": "InputName",
  "Gain": X,
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where OutputName gives a convenient name for the block (i.e. SpeedControl).
Input is the full path name of the input signal
Gain is the gain applied to the input signal
Limits are optional and saturate the output if defined
*/
void ControlGainClass::Configure(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    std::string OutputName;
    if (Config.containsKey("Output")) {
      OutputName = RootPath + "/" + (std::string) Config.get<String>("Output").c_str();
    } else {
      Serial.println("ERROR: Output not specified in configuration.");
      while(1){}
    }
    if (Config.containsKey("Gain")) {
      config_.Gain = Config["Gain"];
    } else {
      Serial.println("ERROR: Gain value not specified in configuration.");
      while(1){}
    }
    if (Config.containsKey("Input")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(Config.get<String>("Input").c_str())) {
        config_.Input = DefinitionTreePtr->GetValuePtr<float*>(Config.get<String>("Input").c_str());
      } else {
        Serial.println("ERROR: Input not found in global data.");
        while(1){}
      }
    } else {
      Serial.println("ERROR: Input not specified in configuration");
      while(1){}
    }
    if (Config.containsKey("Limits")) {
      config_.SaturateOutput = true;
      // pointer to log saturation data
      // DefinitionTreePtr->InitMember(OutputName+"/Saturated",&data_.Saturated);
      JsonObject &Limits = Config["Limits"];
      if (Limits.containsKey("Lower")&&Limits.containsKey("Upper")) {
        config_.UpperLimit = Limits["Upper"];
        config_.LowerLimit = Limits["Lower"];
      } else {
        Serial.println("ERROR: Either upper or lower limit not specified in configuration.");
        while(1){}
      }
    }
    // pointer to log run mode data
    // DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode);
    // pointer to log command data
    DefinitionTreePtr->InitMember(OutputName,&data_.Command);
  }
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
  data_.Command = *config_.Input*config_.Gain;
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

/* configures control laws given a JSON value and registers data with global defs */
void ControlLaws::Configure(const char *JsonString,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  std::vector<char> buffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  buffer.resize(ConfigBuffer.size());
  if (Config.success()) {
    // configuring baseline control laws
    if (Config.containsKey("Fmu")) {
      // getting the group name
      std::string GroupName = (std::string) Config.get<String>("Fmu").c_str();
      // looking in base configuration for the group definition
      if (Config.containsKey(GroupName.c_str())) {
        // configuring our path
        std::string PathName = RootPath_;
        // parsing the array of levels
        JsonArray& BaselineConfig = Config[GroupName.c_str()];
        // resizing the control group levels
        BaselineControlGroup_.resize(BaselineConfig.size());
        // iterating through levels
        for (size_t i=0; i < BaselineConfig.size(); i++) {
          JsonObject& Level = BaselineConfig[i];
          if (Level.containsKey("Level")&&Level.containsKey("Components")) {
            JsonArray& Components = Level["Components"];
            // iterating through components
            for (size_t j=0; j < Components.size(); j++) {
              JsonObject& Component = Components[j];
              if (Component.containsKey("Type")) {
                if (Component["Type"] == "Gain") {
                  Component.printTo(buffer.data(),buffer.size());
                  ControlGainClass Temp;
                  BaselineControlGroup_[i].push_back(std::make_shared<ControlGainClass>(Temp));
                  BaselineControlGroup_[i][j]->Configure(buffer.data(),PathName,DefinitionTreePtr);
                }
              } else {
                Serial.println("ERROR: Control law type not specified in configuration");
                while(1){}
              }
            }
          } else {
            Serial.println("ERROR: Level or components not specified in configuration");
            while(1){}
          }
        }
      } else {
        Serial.println("ERROR: Cannot find baseline control law in configuration");
        while(1){}
      }
    }
  }
}

/* returns the number of levels for the engaged control law */
size_t ControlLaws::ActiveControlLevels() {
  return BaselineControlGroup_.size();
}

/* computes control law data */
void ControlLaws::Run(size_t ControlLevel) {
  for (size_t i=0; i < BaselineControlGroup_[ControlLevel].size(); i++) {
      BaselineControlGroup_[ControlLevel][i]->Run(ControlFunctionClass::kEngage);
  }
}

/* free resources used by control laws */
void ControlLaws::End() {
  BaselineControlGroup_.clear();
}
