/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#include "control.h"

/* configures control laws given a JSON value and registers data with global defs */
void ControlSystem::Configure(const rapidjson::Value& Config) {
  // Load Control Definitions
  if (!Config.HasMember("ControlDef")) { // ControlDef not defined
    throw std::runtime_error(std::string("ERROR - ControlDef not found in Control."));
  }
  const rapidjson::Value& ControlDef = Config["ControlDef"];

  // Load Controller Group Definition
  if (!Config.HasMember("GroupDef")) { // GroupDef not defined
    throw std::runtime_error(std::string("ERROR - GroupDef not found in Control."));
  }
  const rapidjson::Value& GroupDef = Config["GroupDef"];

  // Load Fmu Set of Controller Groups
  if (!Config.HasMember("Fmu")) { // Fmu not defined
    throw std::runtime_error(std::string("ERROR - Fmu Defintion not found in Control."));
  }
  /* const rapidjson::Value& FmuDef = Config["Fmu"]; */

  // Loop through Baseline  Controller Groups
  // Create new instance of each unique Controller encountered
  // deftree entries will be in /Control/Baseline/<Controller1>/...
  // Levels don't really have a function for the Baseline controller, but are retained anyway
  // Controllers are executed in the order defined in the GroupDef.
  // Controller components are executed in the order defined.

  // Load Baseline Set of Controller Groups
  if (!Config.HasMember("Baseline")) { // Baseline not defined
    throw std::runtime_error(std::string("ERROR - Baseline Defintion not found in Control."));
  }
  const rapidjson::Value& BaselineDef = Config["Baseline"];

  BaselinePath_ = RootPath_ + "/Baseline";
  ConfigureSet(BaselinePath_, BaselineDef, GroupDef, ControlDef, &BaselineGroupMap_, &BaselineControlMap_, &BaselineNodeMap_);

  // Loop through Test Controller Groups
  // Create new instance of each unique Controller encountered
  // deftree entries will be in /Control/Test/<Controller1>/...
  // Levels are used to insert excitations.
  // Controllers are executed in the order defined in the GroupDef.
  // Controller components are executed in the order defined.

  // Load Test Set of Controller Groups
  if (!Config.HasMember("Test")) { // Fmu not defined
    throw std::runtime_error(std::string("ERROR - Test Control Defintion not found in Control."));
  }
  const rapidjson::Value& TestDef = Config["Test"];

  TestPath_ = RootPath_ + "/Test";
  ConfigureSet(TestPath_, TestDef, GroupDef, ControlDef, &TestGroupMap_, &TestControlMap_, &TestNodeMap_);

}

void ControlSystem::ConfigureSet(
  std::string SetPath,
  const rapidjson::Value& SetDef, const rapidjson::Value& GroupDef, const rapidjson::Value& ControlDef,
  GroupMap *SetGroupMap, ControlMap *SetControlMap, NodeMap *SetNodeMap)
  {

  GroupVec GroupStructVec;
  // Loop through each of the Groups defined in the Set
  for (rapidjson::Value::ConstValueIterator SetDefInst = SetDef.Begin(); SetDefInst != SetDef.End(); ++SetDefInst) {
    std::string GroupName = (*SetDefInst).GetString();

    // Check that the defined Group Exists in GroupDef
    if (!GroupDef.HasMember(GroupName.c_str())) {
      throw std::runtime_error(std::string("ERROR - ") + GroupName + std::string(" not defined in not found in GroupDef."));
    }

    // Load the Group defintion
    const rapidjson::Value& GroupList = GroupDef[GroupName.c_str()];

    // Loop through each of the defined (Level, Controller) pairs in the Group
    GroupStructVec.clear();
    NodeStruct NodeStructure;
    for (rapidjson::Value::ConstMemberIterator GroupInst = GroupList.MemberBegin(); GroupInst != GroupList.MemberEnd(); ++GroupInst) {
      // Get the (Level, Controller)
      std::string LevelName = GroupInst->name.GetString();
      std::string ControlName = GroupInst->value.GetString();

      // Add the (Level, Controller) to the Vector
      GroupStruct GroupStructInst;
      GroupStructInst.Level = LevelName;
      GroupStructInst.Controller = ControlName;

      GroupStructVec.push_back(GroupStructInst);

      // Complete Path for Controller
      std::string ControlPath = SetPath + "/" + ControlName;

      // Check that the defined Controller Exists in ControlDef
      if (!ControlDef.HasMember(ControlName.c_str())) {
        throw std::runtime_error(std::string("ERROR - ") + GroupName + std::string(" not defined in not found in GroupDef."));
      }

      // Load the Controller defintion
      const rapidjson::Value& ControlDefElem = ControlDef[ControlName.c_str()];

      // Make sure Controller is not alread defined in the Map, only want a single instance
      if ((*SetControlMap).count(ControlName) == 0) {
        // Create a Shared Pointer to an instance of the ComponentWrapper
        ComponentWrapperPtr CompWrapPtr = std::make_shared<ComponentWrapper>();

        // Call the ComponentWrapper Config, this configures the individual components
        CompWrapPtr->Configure(ControlPath, ControlDefElem);

        // Add the ComponentWrapper Pointer to ControlMap
        (*SetControlMap).insert(std::make_pair(ControlName, CompWrapPtr));
      }

      // Get all the Output Nodes, copy up to higher level and root
      std::vector<std::string> KeyVec;
      deftree.GetKeys(ControlPath, &KeyVec);

      // Loop over Keys, create Vectors of Nodes
      NodeVec NodeVecRoot, NodeVecSet, NodeVecCtrl;
      for (size_t iKey = 0; iKey < KeyVec.size(); ++iKey) {

        std::string Key = KeyVec[iKey].substr (KeyVec[iKey].rfind("/") + 1); // Get just the Key signal name

        NodeVecCtrl.push_back(deftree.getElement(ControlPath + "/" + Key));
        NodeVecSet.push_back(deftree.getElement(SetPath + "/" + Key));
        // NodeVecRoot.push_back(deftree.getElement(RootPath_ + "/" + Key));

        // Copy Node defintions
        NodeVecSet.back()->copyFrom(NodeVecCtrl.back());
        // NodeVecRoot.back()->copyFrom(NodeVecCtrl.back());
      }

      // Create the Map of deftree elements, this is to make copying engaged controller outputs faster during runtime
      NodeStructure.Controller = NodeVecCtrl;
      NodeStructure.Set = NodeVecSet;
      NodeStructure.Root = NodeVecRoot;

      // Node Maps: Controller, Set, and Root
      (*SetNodeMap).insert(std::make_pair(ControlName, NodeStructure));

    } // (Level, Controller) pairs in the Group

    // Add the Vector (Level, Controller) to the Group Map
    (*SetGroupMap).insert(std::make_pair(GroupName, GroupStructVec));

  } // Groups defined in the Set
}

void ControlSystem::ConfigureEffectors( std::vector<std::string> EffKeyVec ) {
  for (size_t iKey = 0; iKey < EffKeyVec.size(); ++iKey) {
    std::string EffKey = EffKeyVec[iKey].substr (EffKeyVec[iKey].rfind("/") + 1);
    EffNodeVec_.push_back(deftree.getElement(RootPath_ + "/" + EffKey, true));
    BaselineEffNodeVec_.push_back(deftree.getElement(BaselinePath_ + "/" + EffKey, true));
    TestEffNodeVec_.push_back(deftree.getElement(TestPath_ + "/" + EffKey, true));
  }
}

void ControlSystem::EffectorBaseline(GenericFunction::Mode mode) {
  // If this is the engaged controller copy deftree elements  1 level higher (to: /Control)
  if (mode == GenericFunction::kEngage) {
    for (size_t iNode = 0; iNode < BaselineEffNodeVec_.size(); ++iNode) {
      EffNodeVec_[iNode]->setDouble(BaselineEffNodeVec_[iNode]->getDouble());
    }
  }
}

void ControlSystem::EffectorTest(GenericFunction::Mode mode) {
  // If this is the engaged controller copy deftree elements  1 level higher (to: /Control)
  if (mode == GenericFunction::kEngage) {
    for (size_t iNode = 0; iNode < TestEffNodeVec_.size(); ++iNode) {
      EffNodeVec_[iNode]->setDouble(TestEffNodeVec_[iNode]->getDouble());
    }
  }
}

void ComponentWrapper::Configure (std::string ControlPath, const rapidjson::Value& Controller) {
  // Loop through each Component of the defined Controller
  for (rapidjson::Value::ConstValueIterator ControlInst = Controller.Begin(); ControlInst != Controller.End(); ++ControlInst) {
    const rapidjson::Value& Component = (*ControlInst);

    // Component type
    if (!Component.HasMember("Type")) {
      throw std::runtime_error(std::string("ERROR - ") + ControlPath + std::string(" Type not found in Controller Componet."));
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
    } else if (ComponentType == "PID2") {
      ComponentVec_.push_back(std::make_shared<PID2Class>());
    } else if (ComponentType == "PID") {
      ComponentVec_.push_back(std::make_shared<PIDClass>());
    } else if (ComponentType == "SS") {
      ComponentVec_.push_back(std::make_shared<SSClass>());
    } else if (ComponentType == "Filter") {
      ComponentVec_.push_back(std::make_shared<GeneralFilter>());
    } else if (ComponentType == "PseudoInverse") {
      ComponentVec_.push_back(std::make_shared<PseudoInverseAllocation>());
    } else if (ComponentType == "Tecs") {
      ComponentVec_.push_back(std::make_shared<TecsClass>());
    } else if (ComponentType == "Latch") {
      ComponentVec_.push_back(std::make_shared<LatchClass>());
    } else {
      throw std::runtime_error(std::string("ERROR - ") + ComponentType + std::string(": Controller Component type does not match known types."));
    }

    // Call Configuration for Component
    ComponentVec_.back()->Configure(Component, ControlPath);
  }
}

void ComponentWrapper::Run (GenericFunction::Mode mode) {
  // Loop through the Components
  for (size_t iComponent = 0; iComponent < ComponentVec_.size(); ++iComponent) {

    // Excecute the Component
    ComponentVec_[iComponent]->Run(mode);
  }
}


/* Baseline Controllers */
std::string ControlSystem::GetBaseline() {
  return BaselineGroupSel_;
}

void ControlSystem::SetBaseline(std::string BaselineGroupSel) {
  BaselineGroupSel_ = BaselineGroupSel;
}

// Run the selected Baseline controller group in the specified mode
void ControlSystem::RunBaseline(GenericFunction::Mode mode) {

  GroupVec BaselineGroupVec = BaselineGroupMap_[BaselineGroupSel_];

  // Loop through each level
  for (size_t iVec = 0; iVec < BaselineGroupVec.size(); ++iVec) {
    std::string ControlName = BaselineGroupVec[iVec].Controller;

    // Run the Controller, using the Map of Wrapper Classes.
    BaselineControlMap_[ControlName]->Run(mode);

    // Copy deftree elements 1 level higher (to: /Control/Baseline)
    NodeVec NodeVecCtrl = BaselineNodeMap_[ControlName].Controller;
    NodeVec NodeVecSet = BaselineNodeMap_[ControlName].Set;

    for (size_t iNode = 0; iNode < NodeVecCtrl.size(); ++iNode) {
      NodeVecSet[iNode]->setDouble(NodeVecCtrl[iNode]->getDouble());
    }
  }
}

/* Test Controllers */
std::string ControlSystem::GetTest() {
  return TestGroupSel_;
}

void ControlSystem::SetTest(std::string TestGroupSel) {
  TestGroupSel_ = TestGroupSel;
}

// Get the Test Levels
std::vector<std::string> ControlSystem::GetTestLevels() {
  std::vector<std::string> LevelVec;
  LevelVec.clear();

  GroupVec TestGroupVec = TestGroupMap_[TestGroupSel_];
  for (size_t iVec = 0; iVec < TestGroupVec.size(); ++iVec) {
    LevelVec.push_back (TestGroupVec[iVec].Level);
  }

  return LevelVec;
}

// Get Current Level
std::string ControlSystem::GetLevel() {
  return TestLevelSel_;
}

// Set Current Level
void ControlSystem::SetLevel(std::string TestLevelSel) {
  TestLevelSel_ = TestLevelSel;
}

// Run the selected Test controller group in the specified mode
void ControlSystem::RunTest(GenericFunction::Mode mode) {

  GroupVec TestGroupVec = TestGroupMap_[TestGroupSel_];

  // Loop through each level
  for (size_t iVec = 0; iVec < TestGroupVec.size(); ++iVec) {
    std::string Level = TestGroupVec[iVec].Level;

    // Only Execute the selected level
    if (Level == TestLevelSel_) {
      std::string ControlName = TestGroupVec[iVec].Controller;

      // Run the Controller, using the Map of Wrapper Classes.
      TestControlMap_[ControlName]->Run(mode);

      // Copy deftree elements 1 level higher (to: /Control/Test)
      NodeVec NodeVecCtrl = TestNodeMap_[ControlName].Controller;
      NodeVec NodeVecSet = TestNodeMap_[ControlName].Set;

      for (size_t iNode = 0; iNode < NodeVecCtrl.size(); ++iNode) {
        NodeVecSet[iNode]->setDouble(NodeVecCtrl[iNode]->getDouble());
      }
    }
  }
}
