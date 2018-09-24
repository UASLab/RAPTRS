/*
excitation.cc
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

#include "excitation.hxx"

/* configures excitation system given a JSON value and registers data with global defs */
void ExcitationSystem::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  // the excitation group name
  if (Config.HasMember("Groups")) {
    // making sure time is configured
    if (!Config.HasMember("Time")) {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Time source not specified in configuration."));
    }
    // iterating through the excitation group definitions
    const rapidjson::Value& Groups = Config["Groups"];
    ExcitationGroups_.resize(Groups.Size());
    ExcitationGroupLevels_.resize(Groups.Size());
    for (rapidjson::Value::ConstValueIterator Group = Groups.Begin(); Group != Groups.End(); ++Group) {
      auto GroupIndex = std::distance(Groups.Begin(),Group);
      if (Group->HasMember("Name")&&Group->HasMember("Components")) {
        // excitation group keys
        ExcitationGroupKeys_.push_back((*Group)["Name"].GetString());
        // path name /Excitation/GroupName/
        std::string PathName = RootPath_+"/"+ExcitationGroupKeys_.back();
        const rapidjson::Value& Levels = (*Group)["Components"];
        // iterating through the excitation group levels
        ExcitationGroups_[GroupIndex].resize(Levels.Size());
        for (rapidjson::Value::ConstValueIterator Level = Levels.Begin(); Level != Levels.End(); ++Level) {
          auto LevelIndex = std::distance(Levels.Begin(),Level);
          if (Level->HasMember("Level")&&Level->HasMember("Components")) {
            // level name keys
            ExcitationGroupLevels_[GroupIndex].push_back((*Level)["Level"].GetString());
            const rapidjson::Value& Components = (*Level)["Components"];
            // iterating through excitations
            for (auto &Component : Components.GetArray()) {
              if (Component.HasMember("Waveform")&&Component.HasMember("Signal")&&Component.HasMember("Start-Time")&&Component.HasMember("Scale-Factor")) {
                // checking whether waveform is defined
                if (Config.HasMember(Component["Waveform"].GetString())) {
                  // grabbing the waveform definition
                  const rapidjson::Value& WaveformValue = Config[Component["Waveform"].GetString()];
                  rapidjson::Document Waveform;
                  rapidjson::Document::AllocatorType& Allocator = Waveform.GetAllocator();
                  Waveform.SetObject();
                  // copying the waveform definition into a new JSON document
                  Waveform.CopyFrom(WaveformValue,Allocator);
                  rapidjson::Value Time;
                  rapidjson::Value Signal;
                  // // adding the time, signal, start-time, and scale-factor members
                  Time.SetString(std::string(Config["Time"].GetString()).c_str(),std::string(Config["Time"].GetString()).size(),Allocator);
                  Signal.SetString(std::string(Component["Signal"].GetString()).c_str(),std::string(Component["Signal"].GetString()).size(),Allocator);
                  Waveform.AddMember("Time",Time,Allocator);
                  Waveform.AddMember("Signal",Signal,Allocator);
                  Waveform.AddMember("Start-Time",Component["Start-Time"].GetFloat(),Allocator);
                  Waveform.AddMember("Scale-Factor",Component["Scale-Factor"].GetFloat(),Allocator);
                  if (WaveformValue.HasMember("Type")) {
                    // pushing back the correct waveform type
                    if (WaveformValue["Type"] == "Pulse") {
                      ExcitationGroups_[GroupIndex][LevelIndex].push_back(std::make_shared<Pulse>());
                    } else if (WaveformValue["Type"] == "Doublet") {
                      ExcitationGroups_[GroupIndex][LevelIndex].push_back(std::make_shared<Doublet>());
                    } else if (WaveformValue["Type"] == "Doublet121") {
                      ExcitationGroups_[GroupIndex][LevelIndex].push_back(std::make_shared<Doublet121>());
                    } else if (WaveformValue["Type"] == "Doublet3211") {
                      ExcitationGroups_[GroupIndex][LevelIndex].push_back(std::make_shared<Doublet3211>());
                    } else if (WaveformValue["Type"] == "LinearChirp") {
                      ExcitationGroups_[GroupIndex][LevelIndex].push_back(std::make_shared<LinearChirp>());
                    } else if (WaveformValue["Type"] == "MultiSine") {
                      ExcitationGroups_[GroupIndex][LevelIndex].push_back(std::make_shared<MultiSine>());
                    } else {
                      throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Waveform type does not match known types."));
                    }
                    // configuring the waveform
                    ExcitationGroups_[GroupIndex][LevelIndex].back()->Configure(Waveform,PathName,DefinitionTreePtr);
                  } else {
                    throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Waveform type not specified in definition."));
                  }
                } else {
                  throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Waveform definition not found in configuration."));
                }
              } else {
                throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Waveform, signal, start time or scale factor not specified in configuration."));
              }
            }
          } else {
            throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Level or components not specified in configuration."));
          }
        }
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Group name or components not specified in configuration."));
      }
    }
  }
}

/* sets the engaged excitation group */
void ExcitationSystem::SetEngagedExcitation(std::string ExcitationGroupName) {
  EngagedGroup_ = ExcitationGroupName;
}

/* run all excitation functions at a given control level */
void ExcitationSystem::RunEngaged(std::string ControlLevel) {
  // iterate through all groups
  for (auto Group = ExcitationGroupKeys_.begin(); Group != ExcitationGroupKeys_.end(); ++Group) {
    auto GroupIndex = std::distance(ExcitationGroupKeys_.begin(),Group); 
    // iterate through all levels
    for (auto Level = ExcitationGroupLevels_[GroupIndex].begin(); Level != ExcitationGroupLevels_[GroupIndex].end(); ++Level) {
      auto LevelIndex = std::distance(ExcitationGroupLevels_[GroupIndex].begin(),Level);
      // iterate through all excitations
      for (auto Func : ExcitationGroups_[GroupIndex][LevelIndex]) {
        if (((*Group)==EngagedGroup_)&&((*Level)==ControlLevel)) {
          Func->Run(GenericFunction::kEngage);
        } 
      }
    }
  }
}

void ExcitationSystem::RunArmed() {
  // iterate through all groups
  for (auto Group = ExcitationGroupKeys_.begin(); Group != ExcitationGroupKeys_.end(); ++Group) {
    auto GroupIndex = std::distance(ExcitationGroupKeys_.begin(),Group); 
    // iterate through all levels
    for (auto Level = ExcitationGroupLevels_[GroupIndex].begin(); Level != ExcitationGroupLevels_[GroupIndex].end(); ++Level) {
      auto LevelIndex = std::distance(ExcitationGroupLevels_[GroupIndex].begin(),Level);
      // iterate through all excitations
      for (auto Func : ExcitationGroups_[GroupIndex][LevelIndex]) {
        if ((*Group)!=EngagedGroup_) {
          Func->Run(GenericFunction::kArm);
        } 
      }
    }
  }
}
