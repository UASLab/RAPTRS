/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
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

void ModeSelect::Run(float val, uint8_t *cmdSel, std::string *cmdSelName) {
  uint8_t cmdReq;

  // Loop through the ModeSel_ vector, find the current mode request by value
  cmdReq = 0;
  for (size_t i = 0; i < ModeSel_.size(); ++i) {
      if (val >= ModeSel_[i].Threshold) {
        cmdReq = i;
      } else {
        break;
      }
  }

  // Update and Check for request persistance
  *cmdSel = cmdSelPrev_; // Selected command is the same as previous, unless ...
  if ((cmdSelPrev_ != cmdReq) & (cmdReqPrev_ == cmdReq)) { // request is different than current, and same as previous
    PersistCounter_++;
    if (PersistCounter_ > PersistThreshold_) {
      *cmdSel = cmdReq;
      PersistCounter_ = 0;
    }
  } else {
    PersistCounter_ = 0;
  }

  cmdReqPrev_ = cmdReq;
  cmdSelPrev_ = *cmdSel;

  *cmdSelName = ModeSel_[*cmdSel].Name;
}

/* configures the mission manager given a JSON value and registers data with global defs */
void MissionManager::Configure(const rapidjson::Value& Config) {

  // Throttle Safety Switch congiguration, SOC does not use
  if (!Config.HasMember("Throttle-Safety-Switch")) {
    throw std::runtime_error(std::string("ERROR - Throttle-Safety-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchThrottleConfig = Config["Throttle-Safety-Switch"];

  LoadInput(SwitchThrottleConfig, "Mission-Manager", "Source", &ThrottleSafetySw_.source_node, &ThrottleSafetySw_.sourceKey);
  LoadVal(SwitchThrottleConfig, "Gain", &ThrottleSafetySw_.Gain);
  ThrottleSafetySw_.modeSelectPtr = std::make_shared<ModeSelect>();
  ThrottleSafetySw_.modeSelectPtr->Configure(SwitchThrottleConfig["ModeSel"], 5);

  // SOC switch configuration
  if (!Config.HasMember("Soc-Engage-Switch")) {
    throw std::runtime_error(std::string("ERROR - Soc-Engage-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchSocConfig = Config["Soc-Engage-Switch"];

  LoadInput(SwitchSocConfig, "Mission-Manager", "Source", &SocEngageSw_.source_node, &SocEngageSw_.sourceKey);
  LoadVal(SwitchSocConfig, "Gain", &SocEngageSw_.Gain);
  SocEngageSw_.modeSelectPtr = std::make_shared<ModeSelect>();
  SocEngageSw_.modeSelectPtr->Configure(SwitchSocConfig["ModeSel"], 5);


  // Baseline mode select switch
  if (!Config.HasMember("Baseline-Select-Switch")) {
    throw std::runtime_error(std::string("ERROR - Baseline-Select-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchBaseConfig = Config["Baseline-Select-Switch"];

  LoadInput(SwitchBaseConfig, "Mission-Manager", "Source", &BaseCtrlSelectSw_.source_node, &BaseCtrlSelectSw_.sourceKey);
  LoadVal(SwitchBaseConfig, "Gain", &BaseCtrlSelectSw_.Gain);
  BaseCtrlSelectSw_.modeSelectPtr = std::make_shared<ModeSelect>();
  BaseCtrlSelectSw_.modeSelectPtr->Configure(SwitchBaseConfig["ControlSel"], 5);

  // Test Mode switch configuration
  if (!Config.HasMember("Test-Mode-Switch")) {
    throw std::runtime_error(std::string("ERROR - Test-Mode-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchTestModeConfig = Config["Test-Mode-Switch"];

  LoadInput(SwitchTestModeConfig, "Mission-Manager", "Source", &TestModeSw_.source_node, &TestModeSw_.sourceKey);
  LoadVal(SwitchTestModeConfig, "Gain", &TestModeSw_.Gain);
  TestModeSw_.modeSelectPtr = std::make_shared<ModeSelect>();
  TestModeSw_.modeSelectPtr->Configure(SwitchTestModeConfig["ModeSel"], 5);


  // Test Select switch configuration
  if (!Config.HasMember("Test-Select-Switch")) {
    throw std::runtime_error(std::string("ERROR - Test-Select-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchTestSelConfig = Config["Test-Select-Switch"];

  LoadInput(SwitchTestSelConfig, "Mission-Manager", "Source", &TestSelectSw_.source_node, &TestSelectSw_.sourceKey);
  LoadVal(SwitchTestSelConfig, "Gain", &TestSelectSw_.Gain);
  TestSelectSw_.modeSelectPtr = std::make_shared<ModeSelect>();
  TestSelectSw_.modeSelectPtr->Configure(SwitchTestSelConfig["ModeSel"], 5);


  // Trigger switch configuration
  if (!Config.HasMember("Trigger-Switch")) {
    throw std::runtime_error(std::string("ERROR - Trigger-Switch configuration not found."));
  }
  const rapidjson::Value& SwitchTrigConfig = Config["Trigger-Switch"];

  LoadInput(SwitchTrigConfig, "Mission-Manager", "Source", &TriggerSw_.source_node, &TriggerSw_.sourceKey);
  LoadVal(SwitchTrigConfig, "Gain", &TriggerSw_.Gain);
  TriggerSw_.modeSelectPtr = std::make_shared<ModeSelect>();
  TriggerSw_.modeSelectPtr->Configure(SwitchTrigConfig["ModeSel"], 5);


  // Add signals to the definition tree
  SocEngage_node = deftree.initElement("/Mission/socEngage", "SOC control flag", LOG_UINT8, LOG_NONE);
  BaseCtrlSelect_node = deftree.initElement("/Mission/baseCtrlSel", "Baseline controller selection", LOG_UINT8, LOG_NONE);
  BaseCtrlMode_node = deftree.initElement("/Mission/baseCtrlMode", "Baseline controller mode", LOG_UINT8, LOG_NONE);
  TestCtrlMode_node = deftree.initElement("/Mission/testCtrlMode", "Test controller mode", LOG_UINT8, LOG_NONE);
  TestSenProcMode_node = deftree.initElement("/Mission/testSenProcMode", "Test sensor-processing mode", LOG_UINT8, LOG_NONE);
  TestPtID_node = deftree.initElement("/Mission/testPtID", "Current test point index", LOG_UINT32, LOG_NONE);
  ExciteEngage_node = deftree.initElement("/Mission/excitEngage", "Test engage flag", LOG_UINT8, LOG_NONE);

  // build a map of the test point data
  if (Config.HasMember("Test-Points")) {
    const rapidjson::Value& TestPoints = Config["Test-Points"];
    assert(TestPoints.IsArray());
    NumTestPts_ = TestPoints.Size();
    for (auto &TestPoint : TestPoints.GetArray()) {
      if (TestPoint.HasMember("Test-ID")&&TestPoint.HasMember("Sensor-Processing")&&TestPoint.HasMember("Control")&&TestPoint.HasMember("Excitation")) {
        TestPointDefinition TestPointCurr;

        TestPointCurr.ID = TestPoint["Test-ID"].GetString();
        TestPointCurr.SensorProcessing = TestPoint["Sensor-Processing"].GetString();
        TestPointCurr.Control = TestPoint["Control"].GetString();
        TestPointCurr.Excitation = TestPoint["Excitation"].GetString();

        TestPointsVec_.push_back(TestPointCurr);
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Test-ID, Sensor-Processing, Control, or Excitation not included in test point definition."));
      }
    }
  }
}

/* runs the mission manager */
void MissionManager::Run() {
  /* Switch processing */
  // FMU / SOC switch logic
  uint8_t SocEngageVal;
  std::string SocEngageName;
  SocEngageSw_.modeSelectPtr->Run(SocEngageSw_.source_node->getFloat() * SocEngageSw_.Gain, &SocEngageVal, &SocEngageName);

  // Baseline Control Select
  uint8_t BaseCtrlSelectVal;
  std::string BaseCtrlSelectName;
  BaseCtrlSelectSw_.modeSelectPtr->Run(BaseCtrlSelectSw_.source_node->getFloat() * BaseCtrlSelectSw_.Gain, &BaseCtrlSelectVal, &BaseCtrlSelectName);

  // Research Control law select switch logic
  uint8_t TestModeVal;
  std::string TestModeName;
  TestModeSw_.modeSelectPtr->Run(TestModeSw_.source_node->getFloat() * TestModeSw_.Gain, &TestModeVal, &TestModeName);

  // Test point select logic
  uint8_t TestSelectVal;
  std::string TestSelectName;
  TestSelectSw_.modeSelectPtr->Run(TestSelectSw_.source_node->getFloat() * TestSelectSw_.Gain, &TestSelectVal, &TestSelectName);

  // Trigger switch logic
  uint8_t TriggerVal;
  std::string TriggerName;
  TriggerSw_.modeSelectPtr->Run(TriggerSw_.source_node->getFloat() * TriggerSw_.Gain, &TriggerVal, &TriggerName);


  /* Mode Control Logic */
  // Baseline Modes
  BaseSenProcSel_ = "Baseline"; // Not selectable
  BaseCtrlSel_ = BaseCtrlSelectName;


  // Process trigger Latching
  TriggerAct_ = false;
  if ((TriggerName == "Trigger") & (TriggerLatch_ == false)) {
    TriggerLatch_ = true;
    TriggerAct_ = true;
  } else if (TriggerName == "Standby") {
    TriggerLatch_ = false; // Clear the momentary latch
  }

  // Test Selection
  TestPtID_ = TestPtID_node->getInt(); // Get the currect TestID
  if (TestSelectName == "Excite") { // Excitation selected
    if (TriggerAct_ == true) {
      if (ExciteSel_ == "None") { // Engage the Excitation
        ExciteSel_ = TestPointsVec_[TestPtID_].Excitation;
      } else { // Dis-Engage the Excitation
        ExciteSel_ = "None";
      }
    }
  } else if (TestSelectName == "Increment") { // Increment selected
    ExciteSel_ = "None"; // Dis-Engage the Excitation

    if (TriggerAct_ == true) { // Increment the Test Point, switches engaged controller
      TestPtID_++;
      if (TestPtID_ >= NumTestPts_) {
        TestPtID_ = 0;
      }
    }
  } else if (TestSelectName == "Decrement") { // Decrement selected
    ExciteSel_ = "None"; // Dis-Engage the Excitation

    if (TriggerAct_ == true) { // Decrement the Test Point to 0, switches engaged controller
      TestPtID_ = 0;
    }
  }

  // SOC Controller and SensorProcessing Mode Switching
  if (SocEngageName == "Soc") {
    SocEngageMode_= true;
    TestSenProcSel_ = TestPointsVec_[TestPtID_].SensorProcessing;
    TestCtrlSel_ = TestPointsVec_[TestPtID_].Control;

    if (TestModeName == "Arm") { // SOC Baseline Engaged, Test Armed
      BaseCtrlMode_ = Mode::kEngage; // Engaged
      TestCtrlMode_ = Mode::kArm; // Armed
      ExciteSel_ = "None"; // Dis-Engage the Excitation

    } else if (TestModeName == "Engage") {// SOC Baseline Armed, Test Engaged
      BaseCtrlMode_ = Mode::kArm; // Armed
      TestCtrlMode_ = Mode::kEngage; // Engaged

    } else { // (TestModeName == "Standby") SOC Baseline Engaged, Test Standby
      BaseCtrlMode_ = Mode::kEngage; // Engaged
      TestCtrlMode_ = Mode::kStandby; // Standby
      ExciteSel_ = "None"; // Dis-Engage the Excitation
    }

  } else { // FMU Mode
    SocEngageMode_= false;
    BaseCtrlMode_ = Mode::kArm; // Armed
    TestCtrlMode_ = Mode::kStandby; // Standby
    ExciteSel_ = "None"; // Dis-Engage the Excitation
  }

  // Update Nodes
  SocEngage_node->setBool(SocEngageMode_);
  BaseCtrlSelect_node->setInt(BaseCtrlSelectVal);
  BaseCtrlMode_node->setInt(BaseCtrlMode_);
  TestCtrlMode_node->setInt(TestCtrlMode_);
  TestSenProcMode_node->setInt(TestCtrlMode_);
  TestPtID_node->setInt(TestPtID_);

  if (ExciteSel_ == "None") {
    ExciteEngage_node->setBool(false);
  } else {
    ExciteEngage_node->setBool(true);
  }
}

/* returns the string of the test sensor processing group that is selected */
std::string MissionManager::GetBaselineSensorProcessing() {
  return BaseSenProcSel_;
}

/* returns the run mode of the Baseline System */
GenericFunction::Mode MissionManager::GetBaselineRunMode() {
  return BaseCtrlMode_;
}

/* returns the string of the baseline control group that is armed */
std::string MissionManager::GetBaselineController() {
  return BaseCtrlSel_;
}

/* returns the string of the test sensor processing group that is selected */
std::string MissionManager::GetTestSensorProcessing() {
  return TestSenProcSel_;
}

/* returns the string of the test control group that is selected */
std::string MissionManager::GetTestController() {
  return TestCtrlSel_;
}

/* returns the run mode of the Test System */
GenericFunction::Mode MissionManager::GetTestRunMode() {
  return TestCtrlMode_;
}

/* returns the string of the excitation group that is engaged */
std::string MissionManager::GetExcitation() {
  return ExciteSel_;
}
