/*
mission.hxx
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

#pragma once

#include "hardware-defs.h"
#include "definition-tree2.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "generic-function.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <vector>
#include <cstring>

// Mode change requests are handled with a set of persistance counters.
// Requesting Armed or Engaged states require persistance to switch
class ModeSelect {
  public:
    void Configure(const rapidjson::Value& Config, int PersistThreshold);
    std::string Run(float val);
    void Reset(){
      PersistCounter_ = 0;
    };

    struct ModeEnt {
      std::string Name;
      float Threshold;
    };

  private:
    std::vector<ModeEnt> ModeSel_;
    std::string cmdSel_, cmdReq_;
    int PersistCounter_ = 0;
    int PersistThreshold_ = 0;
};

typedef GenericFunction::Mode Mode;

class MissionManager {
  public:
    struct TestPointDefinition {
      std::string ID;
      std::string SensorProcessing;
      std::string Control;
      std::string Excitation;
    };
    void Configure(const rapidjson::Value& Config);
    void Run();
    std::string GetBaselineSensorProcessing();
    std::string GetBaselineController();
    GenericFunction::Mode GetBaselineRunMode();
    std::string GetTestSensorProcessing();
    std::string GetTestController();
    GenericFunction::Mode GetTestRunMode();
    std::string GetExcitation();
  private:
    std::string RootPath_ = "/Mission-Manager";

    struct Switch {
      ElementPtr source_node;
      std::string sourceKey;
      float Gain = 1.0;
      ModeSelect modeSelect;
    };

    Switch SocEngage_;
    Switch ThrottleSafety_;
    Switch BaselineSelect_;
    Switch TestMode_;
    Switch TestSelect_;
    Switch Trigger_;

    size_t NumberOfTestPoints_ = 0;
    ElementPtr TestPointId_node;

    std::string TestSensorProcessingSel_ = "None";
    std::string TestControlSel_ = "None";
    std::string BaselineSensProcSel_ = "None";
    std::string BaselineControlSel_ = "None";
    Mode BaselineControlMode_ = Mode::kArm; // Armed
    Mode TestControlMode_ = Mode::kStandby; // Standby
    std::string ExcitationSel_ = "None";

    ElementPtr SocEngage_node, BaselineMode_node, TestMode_node, TestSelect_node;
    ElementPtr TestEngageFlag_node;

    std::map<std::string,TestPointDefinition> TestPoints_;
};
