/*
mission.cpp
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
