/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "sensors.h"
#include "effector.h"
#include "Arduino.h"

class AircraftMission {
  public:
    enum State {
      SyncDataCollection,
      AsyncDataCollection,
      EffectorOutput
    };
    enum Mode {
      Configuration,
      Run
    };
    void UpdateMode(AircraftSensors *AircraftSensorsPtr,AircraftEffectors *AircraftEffectorsPtr);
    void SetRequestedMode(Mode &ModeRef);
    void GetMode(Mode *ModePtr);
    void UpdateState();
    void GetState(State *StatePtr);
    void SetSyncDataCollection();
    void ClearSyncDataCollection();
    void SetSyncEffectorOutput();
    void ClearSyncEffectorOutput();
  private:
    Mode CurrentMode_ = Configuration;
    Mode RequestedMode_ = Configuration;
    State state_ = AsyncDataCollection;
    volatile bool SyncDataCollection_ = false;
    volatile bool SyncEffectorOutput_ = false;
};

