/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan, Brian Taylor
*/

#pragma once

#include "excitation-waveforms.h"
#include "configuration.h"

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

/* Class to manage excitations
Example JSON configuration:

"Excitation": {
  "ExciteDef": {
    "RTSM": {"Level": "Alloc", "Waveforms": [
      {"Time": "/Sensors/Fmu/Time_us"},
      {"Wave": "OMS_RTSM_1", "Signal": "/Control/cmdRoll_rps", "Start-Time": 1, "Scale-Factor": 0.0698132},
      {"Wave": "OMS_RTSM_2", "Signal": "/Control/cmdPitch_rps", "Start-Time": 1, "Scale-Factor": 0.0698132},
      {"Wave": "OMS_RTSM_3", "Signal": "/Control/cmdYaw_rps", "Start-Time": 1, "Scale-Factor": 0.0698132}
    ]}
  "WaveDef": {
    "OMS_RTSM_1": {"Type": "MultiSine", "Duration": 10.0,
      "Frequency": [0.6283185307179586, 4.360530603182633, 8.092742675647308, 11.824954748111981, 15.557166820576658, 19.28937889304133, 23.021590965506004, 26.753803037970677, 30.486015110435353, 34.21822718290002, 37.9504392553647, 41.682651327829376, 45.414863400294045, 49.14707547275872, 52.8792875452234, 56.61149961768807, 60.34371169015275],
      "Phase": [6.174200357528524, 5.448037839217776, 3.8454607850193847, 1.6650584957876886, 3.9242402680243402, 4.630080151304789, 5.912432865757586, 5.316652321008593, 2.7342570964241117, 6.546636801464082, 8.454702155760696, 8.24574027481253, 8.722410829390709, 7.878019406341706, 5.298703900733216, 7.7365320512089255, 9.81651753182983],
      "Amplitude": [0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396, 0.19810717087274396]
    }
  }
}

Where:
  * "ExciteDef" is a list of selectable excitation systems.
    Each element of "ExciteDef" is Named.
    Each named element has a "Level" definition that cooresponds to the Controller levels
    Each named element has a "Waveforms" definition consisting of:
      "Time"
      and a number of "Wave" definitions, which correspond to elements of "WaveDef"
  * "WaveDef" is a list of selectable waveforms.
    Each element of "WaveDef" is Named.
    Each named element has a "Type" definition that cooresponds to an Excitation Class.

*/

// Wrapper for Groups of Waveforms
class ExcitationWrapper {
  public:
    void Configure(std::string ExcitePath, const rapidjson::Value& WaveDef, const rapidjson::Value& Waveforms);
    bool Run(float tEngaged_s);

    struct WaveStruct {
      float TimeStart_s;
      float Scale;
      std::string Type;
      ElementPtr NodeSignal;
      ElementPtr NodeExcite;
      std::shared_ptr<Waveform> WaveFunc;
    };

  private:
    std::vector<WaveStruct> WaveVec_;
};

//
class ExcitationSystem {
  public:
    void Configure(const rapidjson::Value& Config);
    void SetExcitation(std::string ExcitEngaged);
    void Run(std::string ControlLevel);
  private:
    std::string RootPath_ = "/Excitation";
    std::string ExcitEngaged_ = "None";
    bool Engaged_ = 0;
    bool Active_ = 0;
    float tStart_s = 0;
    float tEngaged_s = 0;
    ElementPtr time_node_;
    ElementPtr active_node_;
    std::map<std::string, std::string> ExciteLevelMap_;
    std::map<std::string, std::shared_ptr<ExcitationWrapper>> ExciteWrapMap_;
};
