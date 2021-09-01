/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "definition-tree.h"
#include "effector.h"
#include "control.h"
#include "sensors.h"
#include "Arduino.h"

// forward declaration
class AircraftSensors;

class AircraftMission {
  public:
    enum State {
      SyncDataCollection,
      AsyncDataCollection,
      FlightControl,
      EffectorOutput
    };
    enum Mode {
      Configuration,
      Run
    };
    bool UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload, DefinitionTree *DefinitionTreePtr);
    void UpdateMode(AircraftSensors *AircraftSensorsPtr,ControlLaws *ControlLawsPtr,AircraftEffectors *AircraftEffectorsPtr,DefinitionTree *DefinitionTreePtr);
    void SetRequestedMode(Mode ModeCopy);
    void GetMode(Mode *ModePtr);
    void UpdateState();
    void GetState(State *StatePtr);
    void SetImuDataReady();
    void ClearImuDataReady();
    void ClearFlightControlFlag();
    void ClearEffectorOutputFlag();
    bool UseSocEffectorComands();
    bool ThrottleSafed();
  private:
    struct Config {
      uint32_t EffectorOutputPeriod_us = 18000;
      struct Switch {
        float *Source;
        float Gain = 1.0f;
        float Threshold = 0.5f;
      };
      Switch EngageSwitch;
      Switch ThrottleSwitch;
    };
    Config config_;
    Mode CurrentMode_ = Configuration;
    Mode RequestedMode_ = Configuration;
    State state_ = AsyncDataCollection;
    volatile bool ImuDataReady_ = false;
    volatile bool FlightControl_ = false;
    volatile bool EffectorOutput_ = false;
    bool FlightControlLatch_ = false;
    bool EffectorOutputLatch_ = false;
    bool UseSocEffectorComands_ = false;
    bool ThrottleSafed_ = true;
    bool ThrottleSafedLatch_ = false;
    elapsedMicros EffectorOutputTimer_;
};

