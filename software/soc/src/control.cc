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

#include "control.hxx"

/* see control.hxx for configuration details */

/* configures control laws given a JSON value and registers data with global defs */
void ControlLaws::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  std::map<std::string,std::string> OutputKeysMap;
  // configuring Soc control laws, baseline control laws are on FMU
  if (Config.HasMember("Soc")) {
    const rapidjson::Value& SocConfig = Config["Soc"];
    // iterate over Soc control law names
    for (auto &GroupName : SocConfig.GetArray()) {
      // grab Soc control law definition
      if (Config.HasMember(GroupName.GetString())) {
        // store the group key
        SocGroupKeys_.push_back(GroupName.GetString());
        // json group definition
        const rapidjson::Value& GroupDefinition = Config[GroupName.GetString()];
        // resize the control group by the number of levels
        SocControlGroups_[SocGroupKeys_.back()].resize(GroupDefinition.Size());
        // resize the data keys by the number of levels
        SocDataKeys_[SocGroupKeys_.back()].resize(GroupDefinition.Size());
        // iterate over the levels
        for (rapidjson::Value::ConstValueIterator Member = GroupDefinition.Begin(); Member != GroupDefinition.End(); ++Member) {
          if (Member->HasMember("Level")&&Member->HasMember("Components")) {
            auto level = std::distance(GroupDefinition.Begin(),Member);
            // store the level names
            SocLevelNames_[SocGroupKeys_.back()].push_back((*Member)["Level"].GetString());
            // path for the Soc functions /Control/"Group-Name"
            std::string PathName = RootPath_+"/"+SocGroupKeys_.back()+"/"+SocLevelNames_[SocGroupKeys_.back()].back();
            // json components on a given level
            const rapidjson::Value& Components = (*Member)["Components"];
            // iterate over the components on each level
            for (auto &Func : Components.GetArray()) {
              if (Func.HasMember("Type")) {
                if (Func["Type"] == "Constant") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<ConstantClass>());
                }
                if (Func["Type"] == "Gain") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<GainClass>());
                }
                if (Func["Type"] == "Sum") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<SumClass>());
                }
                if (Func["Type"] == "PID2") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<PID2Class>());
                }
                if (Func["Type"] == "PID") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<PIDClass>());
                }
                if (Func["Type"] == "SS") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<SSClass>());
                }
                if (Func["Type"] == "Filter") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<GeneralFilter>());
                }
                if (Func["Type"] == "PseudoInverse") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<PseudoInverseAllocation>());
                }
                if (Func["Type"] == "Tecs") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<TecsClass>());
                }
                if (Func["Type"] == "Latch") {
                  SocControlGroups_[SocGroupKeys_.back()][level].push_back(std::make_shared<LatchClass>());
                }
                // configure the function
                SocControlGroups_[SocGroupKeys_.back()][level].back()->Configure(Func,PathName,DefinitionTreePtr);
              } else {
                throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Type not specified in configuration."));
              }
            }
            // getting a list of all Soc keys and adding to superset of output keys
            // modify the key to remove the intermediate path
            // (i.e. /Control/GroupName/Pitch --> /Control/Pitch)
            DefinitionTreePtr->GetKeys(PathName,&SocDataKeys_[SocGroupKeys_.back()][level]);
            for (auto Key : SocDataKeys_[SocGroupKeys_.back()][level]) {
              std::string MemberName = RootPath_+Key.substr(Key.rfind("/"));
              if ((Key.substr(Key.rfind("/"))!="/Mode")&&(Key.substr(Key.rfind("/"))!="/Saturated")) {
                OutputKeysMap[MemberName] = MemberName;
              }
            }
            /* Soc outputs to superset of outputs */
            // iterate through output keys and check for matching keys in Soc
            for (auto OutputElem : OutputKeysMap) {
              // current output key
              std::string OutputKey = OutputElem.second;
              // iterate through Soc keys
              for (auto GroupKey : SocGroupKeys_) {
                // iterate through all levels
                for (auto Levels = SocLevelNames_[GroupKey].begin(); Levels != SocLevelNames_[GroupKey].end(); ++Levels) {
                  auto Level = std::distance(SocLevelNames_[GroupKey].begin(),Levels);
                  for (auto SocKey : SocDataKeys_[GroupKey][Level]) {
                    // check for a match with output keys
                    if (SocKey.substr(SocKey.rfind("/"))==OutputKey.substr(OutputKey.rfind("/"))) {
                      std::string KeyName = SocKey.substr(SocKey.rfind("/"));
                      // setup Soc data pointer
                      DefinitionTree::VariableDefinition TempDef;
                      DefinitionTreePtr->GetMember(SocKey,&TempDef);
                      SocDataPtr_[GroupKey][KeyName] = TempDef.Value;
                      // check to see if output key has already been registered
                      if (DefinitionTreePtr->Size(OutputKey)==0) {
                        // register output if it has not already been
                        if (DefinitionTreePtr->GetValuePtr<uint64_t*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<uint64_t*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<uint64_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<uint32_t*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<uint32_t*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<uint32_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<uint16_t*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<uint16_t*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<uint16_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<uint8_t*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<uint8_t*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<uint8_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<int64_t*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<int64_t*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<int64_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<int32_t*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<int32_t*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<int32_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<int16_t*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<int16_t*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<int16_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<int8_t*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<int8_t*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<int8_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<float*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<float*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<float>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                        if (DefinitionTreePtr->GetValuePtr<double*>(SocKey)) {
                          OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<double*>(SocKey));
                          DefinitionTreePtr->InitMember(OutputKey,std::get_if<double>(&OutputData_[KeyName]),TempDef.Description,true,false);
                        }
                      } else {

                      }
                    } else {

                    }
                  }
                }
              }
            }
          } else {
            throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Level name or components not specified in configuration."));
          }
        }
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Group name not found in configuration."));
      }
    }
  } else {
    std::cout << "WARNING" << RootPath_ << ": Soc Control configuration not defined." << std::endl;
  }
}

/* sets the control law that is engaged and currently output */
void ControlLaws::SetEngagedController(std::string ControlGroupName) {
  EngagedGroup_ = ControlGroupName;
}

/* sets the control law that is running and computing states to enable a transient free engage */
void ControlLaws::SetArmedController(std::string ControlGroupName) {
  ArmedGroup_ = ControlGroupName;
}

/* returns the number of levels for the engaged control law */
size_t ControlLaws::ActiveControlLevels() {
  if (EngagedGroup_ == "Baseline") {
    return 0;
  } else {
    return SocControlGroups_[EngagedGroup_].size();
  }
}

/* returns the name of the level for the engaged control law */
std::string ControlLaws::GetActiveLevel(size_t ControlLevel) {
  if (EngagedGroup_ == "Baseline") {
    return "";
  } else {
    return SocLevelNames_[EngagedGroup_][ControlLevel];
  }
}

/* computes control law data */
void ControlLaws::RunEngaged(size_t ControlLevel) {
  if (EngagedGroup_ != "Baseline") {
    // running engaged Soc control laws
    for (auto Func : SocControlGroups_[EngagedGroup_][ControlLevel]) {
      Func->Run(GenericFunction::kEngage);
    }
    // output Soc control laws
    for (auto Key : SocDataKeys_[EngagedGroup_][ControlLevel]) {
      std::string KeyName = Key.substr(Key.rfind("/"));
      if ((KeyName!="/Mode")&&(KeyName!="/Saturated")) {
        if (std::get_if<uint64_t*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<uint64_t*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<uint32_t*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<uint32_t*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<uint16_t*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<uint16_t*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<uint8_t*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<uint8_t*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<int64_t*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<int64_t*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<int32_t*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<int32_t*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<int16_t*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<int16_t*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<int8_t*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<int8_t*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<float*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<float*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<double*>(&SocDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<double*>(&SocDataPtr_[EngagedGroup_][KeyName]));
        }
      }
    }
  }
}

/* computes control law data */
void ControlLaws::RunArmed() {
  // iterate through all groups
  for (auto Group : SocGroupKeys_) {
    // iterate through all levels
    for (auto Levels = SocControlGroups_[Group].begin(); Levels != SocControlGroups_[Group].end(); ++Levels) {
      auto Level = std::distance(SocControlGroups_[Group].begin(),Levels);
      // iterate through all functions
      for (auto Func : SocControlGroups_[Group][Level]) {
        // make sure we don't run the engaged group
        if (Group != EngagedGroup_) {
          // run as arm if the armed group, otherwise standby
          if (Group == ArmedGroup_) {
            Func->Run(GenericFunction::kArm);
          } else {
            Func->Run(GenericFunction::kStandby);
          }
        }
      }
    }
  }
}
