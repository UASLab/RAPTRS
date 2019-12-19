/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan, Brian Taylor
*/

#include "excitation.h"

// Excitation Definitions
void ExcitationSystem::Configure(const rapidjson::Value& Config) {

  // Get the time signal
  std::string TimeKey;
  LoadInput(Config, RootPath_, "Time", &time_node_, &TimeKey);

  // Setup Excitation Active Signal
  active_node_ = deftree.initElement(RootPath_ + "/active", "Excitation active flag", LOG_UINT8, LOG_NONE);

  // Load Waveform Definitions
  if (!Config.HasMember("WaveDef")) { // WaveDef not defined
    throw std::runtime_error(std::string("ERROR - WaveDef not found in Excitation."));
  }
  const rapidjson::Value& WaveDef = Config["WaveDef"];

  // Load Excitation Definition
  if (!Config.HasMember("ExciteDef")) { // ExciteDef not defined
    throw std::runtime_error(std::string("ERROR - ExciteDef not found in Excitation."));
  }
  const rapidjson::Value& ExciteDef = Config["ExciteDef"];


  // Loop through each of the defined Excitations
  for (rapidjson::Value::ConstMemberIterator ExciteDefInst = ExciteDef.MemberBegin(); ExciteDefInst != ExciteDef.MemberEnd(); ++ExciteDefInst) {
    // Get the (Key, Object)
    std::string ExciteDefKey = ExciteDefInst->name.GetString();
    const rapidjson::Value& ExciteDefObj = ExciteDefInst->value;

    // Complete Path for Excitation
    std::string ExcitePath = RootPath_ + "/" + ExciteDefKey;

    // Get the Controller Level Key
    std::string ExciteLevel;
    LoadVal(ExciteDefObj, "Level", &ExciteLevel, true);

    // Insert (Key, Level) into the Map
    ExciteLevelMap_.insert(std::make_pair(ExciteDefKey, ExciteLevel));

    // Get the Waveforms Array
    const rapidjson::Value& Waveforms = ExciteDefObj["Waveforms"];

    // Create a Shared Pointer to an instance of the ExcitationWrapper
    std::shared_ptr<ExcitationWrapper> ExcitationWrapperPtr;

    // Call the Wrapper Config
    ExcitationWrapperPtr = std::make_shared<ExcitationWrapper>();
    ExcitationWrapperPtr->Configure(ExcitePath, WaveDef, Waveforms);

    // Add Wrapper Pointer to Map
    ExciteWrapMap_.insert(std::make_pair(ExciteDefKey, ExcitationWrapperPtr));
  }
}

void ExcitationSystem::SetExcitation(std::string ExcitEngaged) {
  ExcitEngaged_ = ExcitEngaged;
}

// Run all excitations at a given control level
void ExcitationSystem::Run(std::string ControlLevel) {
  // Get the current time
  float tCur_s = time_node_->getFloat() / 1e6;
  bool active = false;
  Active_ = false;

  // If "None" clear the latch and the states
  if (ExcitEngaged_ == "None") {
    tStart_s = 0;
    tEngaged_s = 0;
    Engaged_ = 0;
  }

  // Execute the excitation if the level matches
  if (ExciteLevelMap_[ExcitEngaged_] == ControlLevel) {
    // Get time since Engaged
    if (Engaged_ == 0) {
      tStart_s = tCur_s;
      Engaged_ = 1;
    }
    tEngaged_s = tCur_s - tStart_s;

    // Run the Excitation, using the Wrapper Class
    active = ExciteWrapMap_[ExcitEngaged_]->Run(tEngaged_s);
    if (active) {
      Active_ = active;
    }
  }

  // Set the Excitation Active Flag
  active_node_->setInt(Active_);
}


bool ExcitationWrapper::Run(float tEngaged_s) {

  float Excite = 0.0f;
  float ExciteScaled = 0.0f;
  bool active = false;

  // Loop through the Waves
  for (size_t iWave = 0; iWave < WaveVec_.size(); ++iWave) {
    // Excitation time
    float tExcite_s = tEngaged_s - WaveVec_[iWave].TimeStart_s;

    // Excecute the Wave
    if (tExcite_s >= 0.0f) {
      active = WaveVec_[iWave].WaveFunc->Run(tExcite_s, &Excite);
      ExciteScaled = WaveVec_[iWave].Scale * Excite;
    }

    // Apply the Excitation to the Excitation Log
    ElementPtr NodeExcite = WaveVec_[iWave].NodeExcite;
    NodeExcite->setFloat( ExciteScaled );

    // Apply the Excitation to the Signal
    ElementPtr NodeSignal = WaveVec_[iWave].NodeSignal;
    NodeSignal->setFloat( NodeSignal->getFloat() + ExciteScaled );
  }

  return active;
}



// Call Wrapper to combine Excitation and Waveform elements into a vector of Wave Classes
void ExcitationWrapper::Configure(std::string ExcitePath, const rapidjson::Value& WaveDef, const rapidjson::Value& Waveforms) {
  // Loop through each of the defined Waveforms
  for (rapidjson::Value::ConstValueIterator WaveformElem = Waveforms.Begin(); WaveformElem != Waveforms.End(); ++WaveformElem) {
    const rapidjson::Value& WaveformDef = (*WaveformElem);

    WaveStruct WaveStructInst;

    WaveStructInst.TimeStart_s = 0.0f;
    LoadVal(WaveformDef, "Start-Time", &WaveStructInst.TimeStart_s);

    WaveStructInst.Scale = 1.0f;
    LoadVal(WaveformDef, "Scale-Factor", &WaveStructInst.Scale);

    std::string SignalKey;
    LoadInput(WaveformDef, ExcitePath, "Signal", &WaveStructInst.NodeSignal, &SignalKey);

    std::string ExciteKey = ExcitePath + "/" + SignalKey.substr (SignalKey.rfind("/")+1);
    WaveStructInst.NodeExcite = deftree.initElement(ExciteKey, ": Excitation Signal", LOG_FLOAT, LOG_NONE);

    // Get the Wave definition
    std::string WaveKey;
    LoadVal(WaveformDef, "Wave", &WaveKey, true);
    if (!WaveDef.HasMember(WaveKey.c_str())) {
      throw std::runtime_error(std::string("ERROR - ") + WaveKey + std::string(": Wave not found in WaveDef."));
    }
    const rapidjson::Value& Wave = WaveDef[WaveKey.c_str()];

    // Wave type
    std::string WaveType;
    LoadVal(Wave, "Type", &WaveType, true);

    // Create the Wave Object
    if (WaveType == "Pulse") {
      WaveStructInst.WaveFunc = std::make_shared<Pulse>();
    } else if (WaveType == "Doublet") {
      WaveStructInst.WaveFunc = std::make_shared<Doublet>();
    } else if (WaveType == "Doublet121") {
      WaveStructInst.WaveFunc = std::make_shared<Doublet121>();
    } else if (WaveType == "Doublet3211") {
      WaveStructInst.WaveFunc = std::make_shared<Doublet3211>();
    } else if (WaveType == "LinearChirp") {
      WaveStructInst.WaveFunc = std::make_shared<LinearChirp>();
    } else if (WaveType == "LogChirp") {
      WaveStructInst.WaveFunc = std::make_shared<LogChirp>();
    } else if (WaveType == "1-Cos") {
      WaveStructInst.WaveFunc = std::make_shared<Pulse_1_Cos>();
    } else if (WaveType == "MultiSine") {
      WaveStructInst.WaveFunc = std::make_shared<MultiSine>();
    } else if (WaveType == "Sampled") {
      WaveStructInst.WaveFunc = std::make_shared<Sampled>();
    } else {
      throw std::runtime_error(std::string("ERROR - ") + WaveType + std::string(": Wave type does not match known types."));
    }

    // Call Configuration for Wave
    WaveStructInst.WaveFunc->Configure(Wave);

    // Push the working WaveStructInst into the WaveVec
    WaveVec_.push_back(WaveStructInst);
  }
}
