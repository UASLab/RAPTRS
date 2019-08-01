/*
effector.cpp
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

#include "effector.h"

bool AircraftEffectors::UpdateConfig(uint8_t id, uint8_t address, std::vector<uint8_t> *Payload, DefinitionTree *DefinitionTreePtr) {
  if ( id != message::config_effector_id ) {
    // not our message
    return false;
  }
  message::config_effector_t msg;
  msg.unpack(Payload->data(), Payload->size());
  if ( address == 0 ) {
    // local effector
    Data Temp;
    Effectors_.push_back(Temp);
    if ( msg.effector == message::effector_type::motor ) {
      Effectors_.back().Type = kMotor;
      Effectors_.back().SafedCommand = msg.safed_command;
    } else if ( msg.effector == message::effector_type::pwm ) {
      Effectors_.back().Type = kPwm;
    } else if ( msg.effector == message::effector_type::sbus ) {
      Effectors_.back().Type = kSbus;
    }
    if ( DefinitionTreePtr->GetValuePtr<float*>(msg.input.c_str()) ) {
      Effectors_.back().Input = DefinitionTreePtr->GetValuePtr<float*>(msg.input.c_str());
    } else {
      HardFail("ERROR: Effector input not found in global data.");
    }
    Serial.print("config eff cal: ");
    for (int i=0; i < message::max_calibration; i++) {
      Serial.print(i); Serial.print(":"); Serial.print(msg.calibration[i]); Serial.print("  ");
      if ( !isnanf(msg.calibration[i]) ) {
        Effectors_.back().Calibration.push_back(msg.calibration[i]);
      }
    }
    Effectors_.back().Channel = msg.channel;
  } else {
    Serial.print("config node effector: ");
    // node effector
    int found = -1;
    for ( size_t i = 0; i < NodeEffectors_.size(); i++ ) {
      if ( NodeEffectors_[i].Address == address ) {
        found = i;
      }
    }
    if ( found < 0 ){
      Serial.print("creating new entry ");
      found = NodeEffectors_.size();
      NodeEffectors_.push_back(NodeData());
      NodeEffectors_.back().Address = address;
      // create a new node
      NodeEffectors_.back().node = new Node(kBfsPort, NodeEffectors_.back().Address, kBfsRate);
      // start communication with node
      NodeEffectors_.back().node->Begin();
    }
    Serial.print("index: "); Serial.println(found);
    // send config messages
    NodeEffectors_[found].node->SetConfigurationMode();
    if ( msg.effector == message::effector_type::motor ) {
      NodeEffectors_[found].Types.push_back(kMotor);
    } else if ( msg.effector == message::effector_type::pwm ) {
      NodeEffectors_[found].Types.push_back(kPwm);
    } else if ( msg.effector == message::effector_type::sbus ) {
      NodeEffectors_[found].Types.push_back(kSbus);
    }
    NodeEffectors_[found].SafedCommands.push_back(msg.safed_command);
    NodeEffectors_[found].Inputs.push_back(DefinitionTreePtr->GetValuePtr<float*>(msg.input.c_str()));
    NodeEffectors_[found].node->Configure(id, Payload);
  }
}

void AircraftEffectors::Begin() {
  // initialize PWM effectors
  analogWriteResolution(kPwmResolution);
  for (size_t i=0; i < sizeof(kPwmPins); i++) {
    analogWriteFrequency(kPwmPins[i],kPwmFrequency);
  }
  config_.Resolution = powf(2,kPwmResolution) - 1.0f;
  config_.Period = 1.0f/kPwmFrequency * 1000000.0f;
  // initialize SBUS effectors
  sbus_ = new SBUS(kSbusUart);
  sbus_->begin();
  // set nodes to run mode
  for (size_t i=0; i < NodeEffectors_.size(); i++) {
    NodeEffectors_[i].node->SetRunMode();
  }
}

void AircraftEffectors::SetCommands(message::command_effectors_t *msg, bool ThrottleSafed) {
  for ( size_t i = 0; i < Effectors_.size(); i++ ) {
    if (Effectors_[i].Type != kMotor) {
      Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,msg->command[i]);
    } else {
      //Serial.print("safe: "); Serial.println(ThrottleSafed);
      //Serial.print("safed_command: "); Serial.println(Effectors_[i].SafedCommand);
      //Serial.print("cal: ");
      //for ( int i = 0; i < Effectors_[i].Calibration.size(); i++ ) {
      //  Serial.print(Effectors_[i].Calibration[i]); Serial.print(" ");
      //}
      //Serial.println("");
      if (ThrottleSafed) {
        Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,Effectors_[i].SafedCommand);
      } else {
        Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,msg->command[i]);
      }
    }
  }
  size_t EffectorIndex = Effectors_.size();
  for (size_t i=0; i < NodeEffectors_.size(); i++) {
    std::vector<float> NodeCommands;
    for (size_t j=0; j < NodeEffectors_[i].Inputs.size(); j++) {
      if (NodeEffectors_[i].Types[j] != kMotor) {
        NodeCommands.push_back(msg->command[EffectorIndex]);
      } else {
        if (ThrottleSafed) {
          NodeCommands.push_back(NodeEffectors_[i].SafedCommands[j]);
        } else {
          NodeCommands.push_back(msg->command[EffectorIndex]);
        }
      }
      EffectorIndex++;
    }
    NodeEffectors_[i].node->SendEffectorCommand(NodeCommands);
  }
}

void AircraftEffectors::ComputeOutputs(bool ThrottleSafed) {
  for (size_t i=0; i < Effectors_.size(); i++) {
    if (Effectors_[i].Type != kMotor) {
      Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,*Effectors_[i].Input);
    } else {
      //Serial.print("safe: "); Serial.println(ThrottleSafed);
      //Serial.print("safed_command: "); Serial.println(Effectors_[i].SafedCommand);
      //Serial.print("cal: ");
      //for ( int j = 0; j < Effectors_[i].Calibration.size(); j++ ) {
      //  Serial.print(Effectors_[i].Calibration[j]); Serial.print(" ");
      //}
      //Serial.println("");
      if (ThrottleSafed) {
        Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,Effectors_[i].SafedCommand);
      } else {
        Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,*Effectors_[i].Input);
      }
    }
    // Serial.print("eff "); Serial.print(i); Serial.print(" "); Serial.print(*Effectors_[i].Input); Serial.print(" = "); Serial.println(Effectors_[i].Output);
  }
  // Serial.print("do node eff's: "); Serial.println(NodeEffectors_.size());
  for (size_t i=0; i < NodeEffectors_.size(); i++) {
    // Serial.print("node effectors: "); Serial.println(i);
    std::vector<float> NodeCommands;
    for (size_t j=0; j < NodeEffectors_[i].Inputs.size(); j++) {
      if (NodeEffectors_[i].Types[j] != kMotor) {
        NodeCommands.push_back(*NodeEffectors_[i].Inputs[j]);
      } else {
        if (ThrottleSafed) {
          NodeCommands.push_back(NodeEffectors_[i].SafedCommands[j]);
        } else {
          NodeCommands.push_back(*NodeEffectors_[i].Inputs[j]);
        }
      }
    }
    NodeEffectors_[i].node->SendEffectorCommand(NodeCommands);
  }
}

void AircraftEffectors::CommandEffectors() {
  digitalWriteFast(kBfsInt1Pin,LOW);
  digitalWriteFast(kBfsInt2Pin,HIGH);
  uint16_t SbusCmds[16];
  for (size_t i=0; i < Effectors_.size(); i++) {
    if ((Effectors_[i].Type == kPwm)||(Effectors_[i].Type == kMotor)) {
      // Serial.print("pwm: "); Serial.print(i); Serial.print(" "); Serial.print(Effectors_[i].Channel); Serial.print(" = "); Serial.println(Effectors_[i].Output);
      analogWrite(kPwmPins[Effectors_[i].Channel],Effectors_[i].Output/config_.Period*config_.Resolution);
    }
    if (Effectors_[i].Type == kSbus) {
      SbusCmds[Effectors_[i].Channel] = (uint16_t)Effectors_[i].Output;
    }
  }
  sbus_->write(&SbusCmds[0]);
}

void AircraftEffectors::End() {
  Effectors_.clear();
  NodeEffectors_.clear();
  delete sbus_;
  kSbusUart.end();
}
