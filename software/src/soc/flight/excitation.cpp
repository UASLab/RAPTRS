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

#include "excitation.h"

// Excitation Definitions
void ExcitationSystem::Configure(const rapidjson::Value& Config) {

  // Get the time signal
  std::string TimeKey;
  LoadInput(Config, RootPath_, "Time", &time_node_, &TimeKey);

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
  float tCur_s = 0.0f;
  float tLatch_s = 0.0f;

  // Execute the excitation if the level matches
  if (ExciteLevelMap_[ExcitEngaged_] == ControlLevel) {
    // Get the current time, and time since latched in Engaged
    tCur_s = time_node_->getFloat() / 1e6;
    if (Latched_ == 0) {
      tLatch_s = tCur_s;
    }

    // Run the Excitation, using the Wrapper Class
    ExciteWrapMap_[ExcitEngaged_]->Run(tLatch_s);
  }
}




void ExcitationWrapper::Run(float tLatch_s) {

  // Loop through the Waves
  for (size_t iWave = 0; iWave < WaveVec_.size(); ++iWave) {
    // Excitation time
    float tExcite_s = tLatch_s - WaveVec_[iWave].TimeStart_s;

    // Excecute the Wave
    float Excite = 0.0f;
    WaveVec_[iWave].WaveFunc->Run(tExcite_s, &Excite);
    float ExciteScaled = WaveVec_[iWave].Scale * Excite;

    // Apply the Excitation to the Signal
    ElementPtr NodeSignal = WaveVec_[iWave].NodeSignal;
    NodeSignal->setFloat( NodeSignal->getFloat() + ExciteScaled );

    ElementPtr NodeExcite = WaveVec_[iWave].NodeExcite;
    NodeExcite->setFloat( ExciteScaled );
  }
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
    } else {
      throw std::runtime_error(std::string("ERROR - ") + WaveType + std::string(": Wave type does not match known types."));
    }

    // Call Configuration for Wave
    WaveStructInst.WaveFunc->Configure(Wave);

    // Push the working WaveStructInst into the WaveVec
    WaveVec_.push_back(WaveStructInst);
  }
}
