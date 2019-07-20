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

/* updates the mission configuration from a JSON string */
void AircraftMission::UpdateConfig(const char *JsonString,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.containsKey("Fmu-Soc-Switch")) {
    JsonObject &EngageSwitch = Config["Fmu-Soc-Switch"];
    if (EngageSwitch.containsKey("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch.get<String>("Source").c_str())) {
        config_.EngageSwitch.Source = DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch.get<String>("Source").c_str());
      } else {
        Serial.println("ERROR: Engage switch source not found in global data.");
        while(1){}
      }
    } else {
        Serial.println("ERROR: Engage switch source not specified in configuration.");
        while(1){}
    }
    if (EngageSwitch.containsKey("Gain")) {
      config_.EngageSwitch.Gain = EngageSwitch["Gain"];
    }
    if (EngageSwitch.containsKey("Threshold")) {
      config_.EngageSwitch.Threshold = EngageSwitch["Threshold"];
    }
  }
  if (Config.containsKey("Throttle-Safety-Switch")) {
    JsonObject &ThrottleSwitch = Config["Throttle-Safety-Switch"];
    if (ThrottleSwitch.containsKey("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(ThrottleSwitch.get<String>("Source").c_str())) {
        config_.ThrottleSwitch.Source = DefinitionTreePtr->GetValuePtr<float*>(ThrottleSwitch.get<String>("Source").c_str());
      } else {
        Serial.println("ERROR: Throttle safety switch source not found in global data.");
        while(1){}
      }
    } else {
        Serial.println("ERROR: Throttle safety switch source not specified in configuration.");
        while(1){}
    }
    if (ThrottleSwitch.containsKey("Gain")) {
      config_.ThrottleSwitch.Gain = ThrottleSwitch["Gain"];
    }
    if (ThrottleSwitch.containsKey("Threshold")) {
      config_.ThrottleSwitch.Threshold = ThrottleSwitch["Threshold"];
    }
  }
}

/* state machine controlling mode transitions */
void AircraftMission::UpdateMode(AircraftSensors *AircraftSensorsPtr,ControlLaws *ControlLawsPtr,AircraftEffectors *AircraftEffectorsPtr,DefinitionTree *DefinitionTreePtr) {
  if (RequestedMode_!=CurrentMode_) {
    switch (CurrentMode_) {
      case Configuration: {
        switch (RequestedMode_) {
          case Run: {
            // initialize sensors
            AircraftSensorsPtr->Begin();
            // initialize effectors
            AircraftEffectorsPtr->Begin();
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
            // free control laws
            ControlLawsPtr->End();
            // free effectors
            AircraftEffectorsPtr->End();
            // clear global data
            DefinitionTreePtr->Clear();
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
void AircraftMission::SetRequestedMode(Mode ModeCopy) {
  RequestedMode_ = ModeCopy;
}

/* gets the current mode */
void AircraftMission::GetMode(Mode *ModePtr) {
  *ModePtr = CurrentMode_;
}

/* updates the system state */
void AircraftMission::UpdateState() {
  if (ImuDataReady_) {
    state_ = SyncDataCollection;
    EffectorOutputTimer_ = 0;
    FlightControlLatch_ = false;
    ThrottleSafedLatch_ = false;
    EffectorOutputLatch_ = false;
  } else if (FlightControl_) {
    state_ = FlightControl;
  } else if (EffectorOutput_) {
    state_ = EffectorOutput;
  } else {
    state_ = AsyncDataCollection;
  }
  if (*config_.EngageSwitch.Source*config_.EngageSwitch.Gain <= config_.EngageSwitch.Threshold) {
    if (!FlightControlLatch_) {
      UseSocEffectorComands_ = false;
      FlightControl_ = true;
      FlightControlLatch_ = true;
    }
  } else {
    if (!FlightControlLatch_) {
      UseSocEffectorComands_ = true;
      FlightControl_ = false;
      FlightControlLatch_ = true;
    }
  }
  if (*config_.ThrottleSwitch.Source*config_.ThrottleSwitch.Gain > config_.ThrottleSwitch.Threshold) {
    if (!ThrottleSafedLatch_) {
      ThrottleSafed_ = false;
      ThrottleSafedLatch_ = true;
    }
  } else {
    if (!ThrottleSafedLatch_) {
      ThrottleSafed_ = true;
      ThrottleSafedLatch_ = true;
    }
  }
  if (EffectorOutputTimer_ > config_.EffectorOutputPeriod_us) {
    if (!EffectorOutputLatch_) {
      EffectorOutput_ = true;
      EffectorOutputLatch_ = true;
    }
  }
}

/* gets the current state */
void AircraftMission::GetState(State *StatePtr) {
  *StatePtr = state_;
}

/* sets the IMU data ready flag */
void AircraftMission::SetImuDataReady() {
  ImuDataReady_ = true;
}

/* clears the IMU data ready flag */
void AircraftMission::ClearImuDataReady() {
  ImuDataReady_ = false;
}

/* clears the flight control flag */
void AircraftMission::ClearFlightControlFlag() {
  FlightControl_ = false;
}

/* clears the effector output flag */
void AircraftMission::ClearEffectorOutputFlag() {
  EffectorOutput_ = false;
}

/* returns whether FMU or SOC commands should be sent to effectors */
bool AircraftMission::UseSocEffectorComands() {
  return UseSocEffectorComands_;
}

/* returns whether the throttle is safed */
bool AircraftMission::ThrottleSafed() {
  return ThrottleSafed_;
}
