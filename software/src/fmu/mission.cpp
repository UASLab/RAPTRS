/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "mission.h"

/* updates the mission configuration */
bool AircraftMission::UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload, DefinitionTree *DefinitionTreePtr) {
  if ( id != message::config_mission_id ) {
    // not our message
    return false;
  }
  message::config_mission_t msg;
  msg.unpack(Payload->data(), Payload->size());
  if ( msg.switch_name == "Soc-Engage-Switch" ) {
    if (DefinitionTreePtr->GetValuePtr<float*>(msg.source.c_str())) {
      config_.EngageSwitch.Source = DefinitionTreePtr->GetValuePtr<float*>(msg.source.c_str());
    } else {
      HardFail("ERROR: Fmu-soc switch source not found in global data.");
    }
    config_.EngageSwitch.Gain = msg.gain;
    config_.EngageSwitch.Threshold = msg.threshold;
    return true;
  } else if ( msg.switch_name == "Throttle-Safety-Switch" ) {
    Serial.print("mission config throttle safety: ");
    Serial.println(msg.source.c_str());
    if (DefinitionTreePtr->GetValuePtr<float*>(msg.source.c_str())) {
      config_.ThrottleSwitch.Source = DefinitionTreePtr->GetValuePtr<float*>(msg.source.c_str());
    } else {
      HardFail("ERROR: Throttle safety switch source not found in global data.");
    }
    config_.ThrottleSwitch.Gain = msg.gain;
    config_.ThrottleSwitch.Threshold = msg.threshold;
    return true;
  }
  return false;
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
      //Serial.print("src = "); Serial.print(*config_.ThrottleSwitch.Source);
      //Serial.print(" gain = "); Serial.print(config_.ThrottleSwitch.Gain);
      //Serial.print(" thresh = "); Serial.println(config_.ThrottleSwitch.Threshold);
      ThrottleSafed_ = false;
      ThrottleSafedLatch_ = true;
    }
  } else {
    if (!ThrottleSafedLatch_) {
      //Serial.print("src = "); Serial.print(*config_.ThrottleSwitch.Source);
      //Serial.print(" gain = "); Serial.print(config_.ThrottleSwitch.Gain);
      //Serial.print(" thresh = "); Serial.println(config_.ThrottleSwitch.Threshold);
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
