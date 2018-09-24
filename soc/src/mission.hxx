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

#ifndef MISSION_HXX_
#define MISSION_HXX_

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
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
#include <Eigen/Dense>

class MissionManager {
  public:
    struct TestPointDefinition {
      std::string ID;
      std::string SensorProcessing;
      std::string Control;
      std::string Excitation;
    };
    void Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr);
    void Run();
    std::string GetEngagedSensorProcessing();
    std::string GetEngagedController();
    std::string GetArmedController();
    std::string GetEngagedExcitation();
  private:
    struct Configuration {
      struct Switch {
        float *SourcePtr;
        float Threshold = 0.5;
        float Gain = 1.0;
      };
      Switch SocEngageSwitch, CtrlSelectSwitch, TestSelectIncrementSwitch, TestSelectDecrementSwitch, TriggerSwitch, LaunchSelectSwitch, LandSelectSwitch;
      std::string BaselineController, LaunchController, LandController;
    };

    Configuration config_;
    std::string RootPath_ = "/Mission-Manager";
    const size_t PersistenceThreshold_ = 5;

    size_t SocEngagePersistenceCounter_ = 0;
    bool SocEngage_ = false;

    size_t CtrlSelectPersistenceCounter_ = 0;
    bool CtrlSelect_ = false;

    size_t LaunchSelectPersistenceCounter_ = 0;
    size_t LandSelectPersistenceCounter_ = 0;
    size_t BaselineSelectPersistenceCounter_ = 0;
    bool LaunchSelect_ = false;
    bool LandSelect_ = false;
    bool BaselineSelect_ = true;

    size_t TestSelectIncrementPersistenceCounter_ = 0;
    size_t TestSelectDecrementPersistenceCounter_ = 0;
    size_t TestSelectExcitePersistenceCounter_ = 0;
    int TestSelect_ = 0;

    size_t TriggerPersistenceCounter_ = 0;
    bool TriggerLatch_ = false;
    bool Trigger_ = false;

    size_t NumberOfTestPoints_ = 0;
    size_t CurrentTestPointIndex_ = 0;
    size_t NextTestPointIndex_ = 0;

    std::string EngagedSensorProcessing_ = "Baseline";
    std::string EngagedController_ = "Fmu";
    std::string ArmedController_ = "Fmu";
    std::string EngagedExcitation_ = "None";
    bool EngagedExcitationFlag_ = false;

    std::map<std::string,TestPointDefinition> TestPoints_;
};

#endif
