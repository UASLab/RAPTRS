/*
sensor-processing.cc
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

#include "sensor-processing.h"

/* configures sensor processing given a JSON value and registers data with global defs */
void SensorProcessing::Configure(const rapidjson::Value& Config) {
  // Load SensorProcessing System Definitions
  if (!Config.HasMember("Def")) { // ControlDef not defined
    throw std::runtime_error(std::string("ERROR - Def not found in Sensor-Processing."));
  }
  const rapidjson::Value& SystemDef = Config["Def"];


  // Load Fmu Group
  LoadVal(Config, "Fmu", &FmuGroup_, true);


  // Load Baseline System Group
  // Create new instance of each unique Group encountered
  // deftree entries will be in /Sensor-Processing/<System>/...
  // System components are executed in the order defined.
  LoadVal(Config, "Baseline", &BaselineGroup_, true);

  BaselinePath_ = RootPath_ + "/Baseline";

  // Check that the defined Group Exists in SystemDef
  if (!SystemDef.HasMember(BaselineGroup_.c_str())) {
    throw std::runtime_error(std::string("ERROR - ") + BaselineGroup_ + std::string(" not found in Def."));
  }

  // Call the SystemWrapper Config, this configures the individual components
  BaselinePtr_ = std::make_shared<SystemWrapper>();
  BaselinePtr_->Configure(BaselinePath_, SystemDef[BaselineGroup_.c_str()]);


  // Loop through Test System Groups
  // Create new instance of each unique Group encountered
  // deftree entries will be in /Sensor-Processing/<System>/...
  // System components are executed in the order defined.
  std::vector<std::string> TestGroups;
  LoadVal(Config, "Test", &TestGroups, true);

  TestPath_ = RootPath_ + "/Test";

  for (size_t iSystem = 0; iSystem < TestGroups.size(); ++iSystem) {
    std::string GroupName = TestGroups[iSystem];

    // Check that the defined Group Exists in SystemDef
    if (!SystemDef.HasMember(GroupName.c_str())) {
      throw std::runtime_error(std::string("ERROR - ") + GroupName + std::string(" not defined in not found in Def."));
    }

    // Call the SystemWrapper Config, this configures the individual components
    SystemWrapperPtr TestPtr = std::make_shared<SystemWrapper>();
    TestPtr->Configure(TestPath_, SystemDef[GroupName.c_str()]);

    // Add the SystemWrapper Pointer to ControlMap, don't add the System defined in Baseline
    if (GroupName != BaselineGroup_) {
      SystemMap_.insert(std::make_pair(GroupName, TestPtr));
    }
  }
}



void SystemWrapper::Configure (std::string SystemPath, const rapidjson::Value& System) {
  // Loop through each Component of the defined Controller
  for (rapidjson::Value::ConstValueIterator SystemInst = System.Begin(); SystemInst != System.End(); ++SystemInst) {
    const rapidjson::Value& Component = (*SystemInst);

    // Component type
    if (!Component.HasMember("Type")) {
      throw std::runtime_error(std::string("ERROR - ") + SystemPath + std::string(" Type not found in System Component."));
    }
    std::string ComponentType;
    LoadVal(Component, "Type", &ComponentType, true);

    // Create the Component Object, append to vector
    if (ComponentType == "Constant") {
      ComponentVec_.push_back(std::make_shared<ConstantClass>());
    } else if (ComponentType == "Gain") {
      ComponentVec_.push_back(std::make_shared<GainClass>());
    } else if (ComponentType == "Sum") {
      ComponentVec_.push_back(std::make_shared<SumClass>());
    } else if (ComponentType == "Product") {
      ComponentVec_.push_back(std::make_shared<ProductClass>());
    } else if (ComponentType == "Delay") {
      ComponentVec_.push_back(std::make_shared<DelayClass>());
    } else if (ComponentType == "IAS") {
      ComponentVec_.push_back(std::make_shared<IndicatedAirspeed>());
    } else if (ComponentType == "AGL") {
      ComponentVec_.push_back(std::make_shared<AglAltitude>());
    } else if (ComponentType == "PitotStatic") {
      ComponentVec_.push_back(std::make_shared<PitotStatic>());
    } else if (ComponentType == "FiveHole1") {
      ComponentVec_.push_back(std::make_shared<FiveHole1>());
    } else if (ComponentType == "FiveHole2") {
      ComponentVec_.push_back(std::make_shared<FiveHole2>());
    } else if (ComponentType == "EKF15StateINS") {
      ComponentVec_.push_back(std::make_shared<Ekf15StateIns>());
    } else if (ComponentType == "Filter") {
      ComponentVec_.push_back(std::make_shared<GeneralFilter>());
    } else if (ComponentType == "MinCellVolt") {
      ComponentVec_.push_back(std::make_shared<MinCellVolt>());
    } else {
      throw std::runtime_error(std::string("ERROR - ") + ComponentType + std::string(": System Component type does not match known types."));
    }

    // Call Configuration for Component
    ComponentVec_.back()->Configure(Component, SystemPath);
  }
}

void SystemWrapper::Initialize () {
  // Loop through the Components
  for (size_t iComponent = 0; iComponent < ComponentVec_.size(); ++iComponent) {

    // Initialize the Component
    ComponentVec_[iComponent]->Initialize();
  }
}

bool SystemWrapper::Initialized () {
  // Loop through the Components
  for (size_t iComponent = 0; iComponent < ComponentVec_.size(); ++iComponent) {
    // Check the initialized state, return if any are false
    if (!ComponentVec_[iComponent]->Initialized()) {
      return false;
    }
  }
  return true;
}

void SystemWrapper::Run (GenericFunction::Mode mode) {
  // Loop through the Components
  for (size_t iComponent = 0; iComponent < ComponentVec_.size(); ++iComponent) {

    // Excecute the Component
    ComponentVec_[iComponent]->Run(mode);
  }
}

/* initializes sensor processing */
bool SensorProcessing::Initialized() {
  if (InitializedLatch_) {
    return true;
  }

  bool initialized = true;

  // initialize baseline system
  BaselinePtr_->Initialize();

  // Check initialized baseline sensor processing
  if (!BaselinePtr_->Initialized()) {
    initialized = false;
    return initialized;
  }

  // initializing test sensor processing
  for (auto SystemElem : SystemMap_) {
    SystemWrapperPtr TestPtr = SystemElem.second;

    // initialize test system
    TestPtr->Initialize();

    // check initialization
    if (!TestPtr->Initialized()) {
      initialized = false;
      return initialized;
    }
  }

  if (initialized) {
    InitializedLatch_ = true;
  }
  return initialized;
}

/* Run the Baseline sensor processing data */
void SensorProcessing::RunBaseline(GenericFunction::Mode mode) {
  BaselinePtr_->Run(mode);

  // Copy the output up one level, this may get overwritten later by a test system  (to: /Sensor-Processing)
  std::vector<std::string> KeyVec;
  deftree.GetKeys(BaselinePath_, &KeyVec);

  for (size_t iKey = 0; iKey < KeyVec.size(); ++iKey) {
    std::string Key = KeyVec[iKey];

    ElementPtr base_node = deftree.getElement(BaselinePath_ + "/" + Key); // fixit - store during config
    ElementPtr root_node = deftree.getElement(RootPath_ + "/" + Key); // fixit - store during config
    root_node->copyFrom(base_node);
  }
}


std::string SensorProcessing::GetTest() {
  return TestSel_;
}

void SensorProcessing::SetTest(std::string TestSel) {
  TestSel_ = TestSel;
}

/* computes sensor processing data */
void SensorProcessing::RunTest(GenericFunction::Mode mode) {

  // Catch if the Selected system is same as Baseline.
  if (TestSel_ == BaselineGroup_) { // Do Nothing the Requested System is the Baseline (already Ran)
    return;
  }

  // run test system
  SystemMap_[TestSel_]->Run(mode);

  // If Test is Armed or Engaged copy the output up one level
  if ((mode == GenericFunction::Mode::kArm) | (mode == GenericFunction::Mode::kEngage)) {

    std::string SystemPath = TestPath_ + '/' + TestSel_;

    // Copy deftree elements 1 level higher (to: /Sensor-Processing/Test)
    std::vector<std::string> KeyVec;
    deftree.GetKeys(SystemPath, &KeyVec);

    for (size_t iKey = 0; iKey < KeyVec.size(); ++iKey) {
      std::string Key = KeyVec[iKey];

      ElementPtr sys_node = deftree.getElement(SystemPath + "/" + Key); // fixit - store during config
      ElementPtr test_node = deftree.getElement(TestPath_ + "/" + Key); // fixit - store during config
      test_node->copyFrom(sys_node);

      // If Test is Engaged copy deftree elements 1 level higher (to: /Sensor-Processing)
      if (mode == GenericFunction::Mode::kEngage) {
        ElementPtr root_node = deftree.getElement(RootPath_ + "/" + Key); // fixit - store during config
        root_node->copyFrom(test_node);
      }
    }
  }
}
