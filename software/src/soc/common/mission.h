/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "configuration.h"
#include "definition-tree2.h"
#include "rapidjson/document.h"

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
#include <memory>

// Mode change requests are handled with a set of persistance counters.
// Requesting Armed or Engaged states require persistance to switch
class ModeSelect {
  public:
    void Configure(const rapidjson::Value& Config, int PersistThreshold);
    void Run(float val, uint8_t *cmdSel, std::string *cmdSelName);
    void Reset(){
      PersistCounter_ = 0;
    };

    struct ModeEnt {
      std::string Name;
      float Threshold;
    };

  private:
    std::vector<ModeEnt> ModeSel_;
    uint8_t cmdSelPrev_, cmdReqPrev_;
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
    Mode GetBaselineRunMode();
    std::string GetTestSensorProcessing();
    std::string GetTestController();
    Mode GetTestRunMode();
    std::string GetExcitation();
  private:
    std::string RootPath_ = "/Mission-Manager";

    // Switches
    struct Switch {
      ElementPtr source_node;
      std::string sourceKey;
      float Gain = 1.0;
      std::shared_ptr<ModeSelect> modeSelectPtr;
    };

    Switch ThrottleSafetySw_, SocEngageSw_, BaseCtrlSelectSw_, TestModeSw_, TestSelectSw_, TriggerSw_;

    // defTree Elements
    ElementPtr SocEngage_node;
    ElementPtr BaseCtrlSelect_node, BaseCtrlMode_node, TestCtrlMode_node, TestSenProcMode_node;
    ElementPtr TestPtID_node, ExciteEngage_node;

    // Test Point
    size_t TestPtID_ = 0;
    size_t NumTestPts_ = 0;
    std::map<std::string, TestPointDefinition> TestPoints_;

    bool SocEngageMode_ = false;

    std::string BaseSenProcSel_ = "None";
    std::string BaseCtrlSel_ = "None";
    Mode BaseCtrlMode_ = Mode::kArm; // Armed

    std::string TestSenProcSel_ = "None";
    std::string TestCtrlSel_ = "None";
    Mode TestCtrlMode_ = Mode::kStandby; // Standby

    std::string ExciteSel_ = "None";
    bool TriggerLatch_ = false;
    bool TriggerAct_ = false;

};
