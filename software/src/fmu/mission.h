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

#include "definition-tree.h"
#include "effector.h"
#include "control.h"
#include "sensors.h"
#include "../common/ArduinoJson.h"
#include "Arduino.h"

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
    void UpdateConfig(const char *JsonString,DefinitionTree *DefinitionTreePtr);
    void UpdateMode(AircraftSensors *AircraftSensorsPtr,ControlLaws *ControlLawsPtr,AircraftEffectors *AircraftEffectorsPtr,DefinitionTree *DefinitionTreePtr);
    void SetRequestedMode(Mode &ModeRef);
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

#endif
