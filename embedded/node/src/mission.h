/*
mission.h
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

#ifndef MISSION_H_
#define MISSION_H_

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

#endif
