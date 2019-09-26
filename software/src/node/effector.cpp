/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "effector.h"

/* handle effector configuration messages */
bool AircraftEffectors::UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload) {
  if ( id == message::config_effector_id ) {
    Serial.print("Configuring effector: ");
    if ( sbus_ == NULL ) {
      sbus_ = new SBUS(kSbusUart);
      sbus_->begin();
    }
    Data eff;
    message::config_effector_t msg;
    msg.unpack(Payload->data(), Payload->size());
    if ( msg.effector == message::effector_type::pwm or msg.effector == message::effector_type::motor ) {
      eff.Type = kPwm;
    } else if ( msg.effector == message::effector_type::sbus ) {
      eff.Type = kSbus;
    }
    for (int i = 0; i < message::max_calibration; i++) {
      if ( !isnanf(msg.calibration[i]) ) {
        eff.Calibration.push_back(msg.calibration[i]);
      }
    }
    eff.Channel = msg.channel;
    Serial.println(eff.Channel);
    analogWriteResolution(kPwmResolution);
    analogWriteFrequency(kPwmPins[eff.Channel],kPwmFrequency);
    config_.Resolution = powf(2,kPwmResolution) - 1.0f;
    config_.Period = 1.0f/kPwmFrequency * 1000000.0f;
    Effectors_.push_back(eff); // keep!
    return true;
  }
  return false;
}

/* sets the effector angle commands to the given values */
void AircraftEffectors::SetCommands(std::vector<float> Commands) {
  EffectorCommands_ = Commands;
}

/* computes outputs from effector angle commands and a set of calibration polynomials from config */
void AircraftEffectors::ComputeOutputs() {
  for (size_t i=0; i < Effectors_.size(); i++) {
    Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,EffectorCommands_[i]);
  }
}

/* commands the PWM and SBUS effectors */
void AircraftEffectors::CommandEffectors() {
  if (sbus_) {
    uint16_t SbusCmds[16];
    for (size_t i=0; i < Effectors_.size(); i++) {
      if (Effectors_[i].Type == kPwm) {
        analogWrite(kPwmPins[Effectors_[i].Channel],Effectors_[i].Output/config_.Period*config_.Resolution);
      }
      if (Effectors_[i].Type == kSbus) {
        SbusCmds[Effectors_[i].Channel] = (uint16_t)Effectors_[i].Output;
      }
    }
    sbus_->write(&SbusCmds[0]);
  }
}

/* frees the resources used */
void AircraftEffectors::End() {
  Effectors_.clear();
  EffectorCommands_.clear();
  delete sbus_;
  sbus_ = NULL;
  kSbusUart.end();
}
