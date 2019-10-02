/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "mission.h"

/* state machine controlling mode transitions */
void AircraftMission::UpdateMode(AircraftSensors *AircraftSensorsPtr,AircraftEffectors *AircraftEffectorsPtr) {
  if (RequestedMode_!=CurrentMode_) {
    switch (CurrentMode_) {
      case Configuration: {
        switch (RequestedMode_) {
          case Run: {
            // initialize sensors
            AircraftSensorsPtr->Begin();
            // set mode
            Serial.println("Entering run mode...");
            CurrentMode_ = RequestedMode_;
          }
          default: {

          }
        }
      }
      case Run: {
        switch (RequestedMode_) {
          case Configuration: {
            // free sensors
            AircraftSensorsPtr->End();
            // free effectors
            AircraftEffectorsPtr->End();
            // set mode
            Serial.println("Entering configuration mode...");
            CurrentMode_ = RequestedMode_;
          }
          default: {

          }
        }
      }
    }
  }
}

/* sets the requested mode */
void AircraftMission::SetRequestedMode(Mode &ModeRef) {
  RequestedMode_ = ModeRef;
}

/* gets the current mode */
void AircraftMission::GetMode(Mode *ModePtr) {
  *ModePtr = CurrentMode_;
}

/* updates the system state */
void AircraftMission::UpdateState() {
  if (SyncDataCollection_) {
    state_ = SyncDataCollection;
  } else if (SyncEffectorOutput_) {
    state_ = EffectorOutput;
  } else {
    state_ = AsyncDataCollection;
  }
}

/* gets the current state */
void AircraftMission::GetState(State *StatePtr) {
  *StatePtr = state_;
}

/* sets the sync data collection flag */
void AircraftMission::SetSyncDataCollection() {
  SyncDataCollection_ = true;
}

/* clears the sync data collection flag */
void AircraftMission::ClearSyncDataCollection() {
  SyncDataCollection_ = false;
}

/* sets the sync effector output flag */
void AircraftMission::SetSyncEffectorOutput() {
  SyncEffectorOutput_ = true;
}

/* clears the sync effector output flag */
void AircraftMission::ClearSyncEffectorOutput() {
  SyncEffectorOutput_ = false;
}
