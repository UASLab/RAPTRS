/*
mission.cc
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

#include "mission.h"

void ModeSelect::Configure(const rapidjson::Value& Config, int PersistThreshold) {
  ModeEnt modeEnt;

  for (rapidjson::Value::ConstMemberIterator ConfigEnt = Config.MemberBegin(); ConfigEnt != Config.MemberEnd(); ++ConfigEnt) {
    // Get the (Controller, Threshold)
    modeEnt.Name = ConfigEnt->name.GetString();
    modeEnt.Threshold = ConfigEnt->value.GetFloat();

    ModeSel_.push_back(modeEnt);
  }

  PersistThreshold_ = PersistThreshold;
};

std::string ModeSelect::Run(float val) {
  std::string cmdReq;
  std::string cmdSel;

  for (auto const &ModeSelEnt : ModeSel_) {
    std::string ModeSelName = ModeSelEnt.Name;
    float ModeSelThreshold = ModeSelEnt.Threshold;

    if (val >= ModeSelThreshold) {
      cmdReq = ModeSelName;
    }
  }

  cmdSel = cmdSel_; // Selected command is the same as previous, unless ...
  if ((cmdSel_ != cmdReq) & (cmdReq_ == cmdReq)) { // request is different than current, and same as previous
    PersistCounter_++;
    if (PersistCounter_ > PersistThreshold_) {
      cmdSel = cmdReq;
      PersistCounter_ = 0;
    }
  } else {
    PersistCounter_ = 0;
  }

  cmdReq_ = cmdSel;
  cmdSel_ = cmdSel;

  return cmdSel;
}

/* configures the mission manager given a JSON value and registers data with global defs */
void MissionManager::Configure(const rapidjson::Value& Config) {
  // SOC switch configuration
  if (!Config.HasMember("Soc-Engage-Switch")) {
    throw std::runtime_error(std::string("ERROR - Soc-Engage-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchSocConfig = Config["Soc-Engage-Switch"];

  LoadInput(SwitchSocConfig, "Mission-Manager", "Source", SocEngage_.source_node, &SocEngage_.sourceKey);
  LoadVal(SwitchSocConfig, "Gain", &SocEngage_.Gain);
  SocEngage_.modeSelect.Configure(SwitchSocConfig["ModeSel"], 5);


  // Throttle Safety Switch congiguration, SOC does not use
  if (!Config.HasMember("Throttle-Safety-Switch")) {
    throw std::runtime_error(std::string("ERROR - Throttle-Safety-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchThrottleConfig = Config["Throttle-Safety-Switch"];

  LoadInput(SwitchThrottleConfig, "Mission-Manager", "Source", ThrottleSafety_.source_node, &ThrottleSafety_.sourceKey);
  LoadVal(SwitchThrottleConfig, "Gain", &ThrottleSafety_.Gain);
  ThrottleSafety_.modeSelect.Configure(SwitchThrottleConfig["ModeSel"], 5);


  // Baseline mode select switch
  if (!Config.HasMember("Baseline-Select-Switch")) {
    throw std::runtime_error(std::string("ERROR - Baseline-Select-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchBaseConfig = Config["Baseline-Select-Switch"];

  LoadInput(SwitchBaseConfig, "Mission-Manager", "Source", BaselineSelect_.source_node, &BaselineSelect_.sourceKey);
  LoadVal(SwitchBaseConfig, "Gain", &BaselineSelect_.Gain);
  BaselineSelect_.modeSelect.Configure(SwitchBaseConfig["ControlSel"], 5);

  // Test Mode switch configuration
  if (!Config.HasMember("Test-Mode-Switch")) {
    throw std::runtime_error(std::string("ERROR - Test-Mode-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchTestModeConfig = Config["Test-Mode-Switch"];

  LoadInput(SwitchTestModeConfig, "Mission-Manager", "Source", TestMode_.source_node, &TestMode_.sourceKey);
  LoadVal(SwitchTestModeConfig, "Gain", &TestMode_.Gain);
  TestMode_.modeSelect.Configure(SwitchTestModeConfig["ModeSel"], 5);


  // Test Select switch configuration
  if (!Config.HasMember("Test-Select-Switch")) {
    throw std::runtime_error(std::string("ERROR - Test-Select-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchTestSelConfig = Config["Test-Select-Switch"];

  LoadInput(SwitchTestSelConfig, "Mission-Manager", "Source", TestSelect_.source_node, &TestSelect_.sourceKey);
  LoadVal(SwitchTestSelConfig, "Gain", &TestSelect_.Gain);
  TestSelect_.modeSelect.Configure(SwitchTestSelConfig["ModeSel"], 5);


  // Trigger switch configuration
  if (!Config.HasMember("Trigger-Switch")) {
    throw std::runtime_error(std::string("ERROR - Trigger-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchTrigConfig = Config["Trigger-Switch"];

  LoadInput(SwitchTrigConfig, "Mission-Manager", "Source", Trigger_.source_node, &Trigger_.sourceKey);
  LoadVal(SwitchTrigConfig, "Gain", &Trigger_.Gain);
  Trigger_.modeSelect.Configure(SwitchTrigConfig["ModeSel"], 5);


  // Add signals to the definition tree
  TestPointId_node = deftree.initElement("/Mission/testID", "Current test point index", LOG_UINT32, LOG_NONE);
  SocEngage_node = deftree.initElement("/Mission/socEngage", "SOC control flag", LOG_UINT8, LOG_NONE);
  BaselineMode_node = deftree.initElement("/Mission/baseSel", "Baseline control mode", LOG_UINT8, LOG_NONE);
  TestMode_node = deftree.initElement("/Mission/ctrlSel", "Test mode", LOG_UINT8, LOG_NONE);
  TestSelect_node = deftree.initElement("/Mission/testSel", "Test selection", LOG_UINT8, LOG_NONE);
  TestEngageFlag_node = deftree.initElement("/Mission/excitEngage", "Test engage flag", LOG_UINT8, LOG_NONE);

  // build a map of the test point data
  if (Config.HasMember("Test-Points")) {
    const rapidjson::Value& TestPoints = Config["Test-Points"];
    assert(TestPoints.IsArray());
    NumberOfTestPoints_ = TestPoints.Size();
    for (auto &TestPoint : TestPoints.GetArray()) {
      if (TestPoint.HasMember("Test-ID")&&TestPoint.HasMember("Sensor-Processing")&&TestPoint.HasMember("Control")&&TestPoint.HasMember("Excitation")) {
        TestPoints_[TestPoint["Test-ID"].GetString()].ID = TestPoint["Test-ID"].GetString();
        TestPoints_[TestPoint["Test-ID"].GetString()].SensorProcessing = TestPoint["Sensor-Processing"].GetString();
        TestPoints_[TestPoint["Test-ID"].GetString()].Control = TestPoint["Control"].GetString();
        TestPoints_[TestPoint["Test-ID"].GetString()].Excitation = TestPoint["Excitation"].GetString();
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Test-ID, Sensor-Processing, Control, or Excitation not included in test point definition."));
      }
    }
  }
}

/* runs the mission manager */
void MissionManager::Run() {
  // Switch processing

  // FMU / SOC switch logic
  std::string SocEngageName = SocEngage_.modeSelect.Run((SocEngage_.source_node->getFloat()) * SocEngage_.Gain);
  if (SocEngageName == "Soc") {
    SocEngage_node->setBool(true);
  } else {
    SocEngage_node->setBool(false);
  }

  // Baseline Control Select
  std::string BaselineSelectName = BaselineSelect_.modeSelect.Run((BaselineSelect_.source_node->getFloat()) * BaselineSelect_.Gain);

  // Research Control law select switch logic
  std::string TestModeName = TestMode_.modeSelect.Run((TestMode_.source_node->getFloat()) * TestMode_.Gain);
  if (TestModeName == "Standby") {
    TestMode_node->setInt(0);
  } else if (TestModeName == "Arm") {
    TestMode_node->setInt(1);
  } else if (TestModeName == "Engage") {
    TestMode_node->setInt(3);
  }


  // Test point select logic
  std::string TestSelectName = TestSelect_.modeSelect.Run((TestSelect_.source_node->getFloat()) * TestSelect_.Gain);
  if (TestSelectName == "Decrement") {
    TestSelect_node->setInt(-1);
  } else if (TestSelectName == "Excite") {
    TestSelect_node->setInt(0);
  } else if (TestSelectName == "Increment") {
    TestSelect_node->setInt(1);
  }


  // Trigger switch logic
  std::string TriggerCmd = Trigger_.modeSelect.Run((Trigger_.source_node->getFloat()) * Trigger_.Gain);
  bool TriggerAct;
  if (TriggerCmd == "Standby") {
    TriggerAct == false;
  } else if (TriggerCmd == "Excite") {
    TriggerAct == true;
    Trigger_.modeSelect.Reset();
  }

  // Mode Control Logic

  // Test Selection
  if (TestSelect_node->getInt() == 0) { // Excitation selected
    if (TriggerAct == true) {
      if (ExcitationSel_ == "None") { // Engage the Excitation
        ExcitationSel_ = TestPoints_[std::to_string(TestPointId_node->getInt())].Excitation;
      } else { // Dis-Engage the Excitation
        ExcitationSel_ = "None";
      }
      TriggerAct = false;
    }
  } else if (TestSelect_node->getInt() == 1) { // Increment selected
    ExcitationSel_ = "None";

    if (TriggerAct == true) { // Increment the Test Point, switches engaged controller
      int next = TestPointId_node->getInt() + 1;

      if (next >= NumberOfTestPoints_) {
        next = 0;
      }

      TestPointId_node->setInt(next);
      TriggerAct = false;
    }
  } else if (TestSelect_node->getInt() == -1) { // Decrement selected
    ExcitationSel_ = "None";

    if (TriggerAct == true) { // Decrement the Test Point to 0, switches engaged controller
      TestPointId_node->setInt(0);
      TriggerAct = false;
    }
  }

  // SOC Controller and SensorProcessing Mode Switching
  BaselineSensProcSel_ = "Baseline";
  BaselineControlSel_ = BaselineSelectName;

  if (SocEngage_node->getBool() == true) {
    TestSensorProcessingSel_ = TestPoints_[std::to_string(TestPointId_node->getInt())].SensorProcessing;
    TestControlSel_ = TestPoints_[std::to_string(TestPointId_node->getInt())].Control;

    if (TestMode_node->getInt() == Mode::kArm) { // SOC Baseline Engaged, Test Armed
      BaselineControlMode_ = Mode::kEngage; // Engaged

      TestControlMode_ = Mode::kArm; // Armed
      ExcitationSel_ = TestPoints_[std::to_string(TestPointId_node->getInt())].Excitation;

    } else if (TestMode_node->getInt() == Mode::kEngage) {// SOC Baseline Armed, Test Engaged
      BaselineControlMode_ = Mode::kArm; // Armed

      TestControlMode_ = Mode::kEngage; // Engaged
      ExcitationSel_ = TestPoints_[std::to_string(TestPointId_node->getInt())].Excitation;

    } else { // (TestMode_node->getInt() == 0) SOC Baseline Engaged, Test Standby
      BaselineControlMode_ = Mode::kEngage; // Engaged
      TestControlMode_ = Mode::kStandby; // Standby
      ExcitationSel_ = "None";
    }

  } else { // FMU Mode
    BaselineControlMode_ = Mode::kArm; // Armed
    TestControlMode_ = Mode::kStandby; // Standby
    ExcitationSel_ = "None";
  }

  if (ExcitationSel_ == "None") {
    TestEngageFlag_node->setBool(false);
  } else {
    TestEngageFlag_node->setBool(true);
  }
}

/* returns the string of the test sensor processing group that is selected */
std::string MissionManager::GetBaselineSensorProcessing() {
  return BaselineSensProcSel_;
}

/* returns the run mode of the Baseline System */
GenericFunction::Mode MissionManager::GetBaselineRunMode() {
  return BaselineControlMode_;
}

/* returns the string of the baseline control group that is armed */
std::string MissionManager::GetBaselineController() {
  return BaselineControlSel_;
}

/* returns the string of the test sensor processing group that is selected */
std::string MissionManager::GetTestSensorProcessing() {
  return TestSensorProcessingSel_;
}

/* returns the string of the test control group that is selected */
std::string MissionManager::GetTestController() {
  return TestControlSel_;
}

/* returns the run mode of the Test System */
GenericFunction::Mode MissionManager::GetTestRunMode() {
  return TestControlMode_;
}

/* returns the string of the excitation group that is engaged */
std::string MissionManager::GetExcitation() {
  return ExcitationSel_;
}
