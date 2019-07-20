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

void AircraftEffectors::UpdateConfig(const char *JsonString,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  std::vector<char> buffer;
  JsonArray &Config = ConfigBuffer.parseArray(JsonString);
  buffer.resize(ConfigBuffer.size());
  if (Config.success()) {
    for (size_t i=0; i < Config.size(); i++) {
      JsonObject& Effector = Config[i];
      if ((Effector["Type"] == "Motor") || (Effector["Type"] == "Pwm") || (Effector["Type"] == "Sbus")) {
        Data Temp;
        Effectors_.push_back(Temp);

        if (Effector["Type"] == "Motor") {
          Effectors_.back().Type = kMotor;
          if (Effector.containsKey("Safed-Command")) {
            Effectors_.back().SafedCommand = Effector["Safed-Command"];
          }
        }
        if (Effector["Type"] == "Pwm") {
          Effectors_.back().Type = kPwm;
        }
        if (Effector["Type"] == "Sbus") {
          Effectors_.back().Type = kSbus;
        }

        if (Effector.containsKey("Input")) {
          if (DefinitionTreePtr->GetValuePtr<float*>(Effector.get<String>("Input").c_str())) {
            Effectors_.back().Input = DefinitionTreePtr->GetValuePtr<float*>(Effector.get<String>("Input").c_str());
          } else {
            while(1) {
              Serial.println("ERROR: Input not found in global data.");
              Serial.println(Effector.get<String>("Input"));
            }
          }
        } else {
          while(1){
            Serial.println("ERROR: Input not specified in configuration");
          }
        }

        if (Effector.containsKey("Calibration")) {
          JsonArray &Calibration = Effector["Calibration"];
          for (size_t i=0; i < Calibration.size(); i++) {
            Effectors_.back().Calibration.push_back(Calibration[i]);
          }
        } else {
          while(1){
            Serial.println("ERROR: Calibration not specified in configuration");
          }
        }

        if (Effector.containsKey("Channel")) {
          Effectors_.back().Channel = Effector["Channel"];
        } else {
          while(1){
            Serial.println("ERROR: Channel not specified in configuration");
          }
        }
      }

      if (Effector["Type"] == "Node") {
        NodeData Temp;
        NodeEffectors_.push_back(Temp);
        if (Effector.containsKey("Address")) {
          NodeEffectors_.back().Address = Effector["Address"];
        } else {
          while(1){
            Serial.println("ERROR: Node address not specified in configuration");
          }
        }
        // create a new node
        NodeEffectors_.back().node = new Node(kBfsPort,NodeEffectors_.back().Address,kBfsRate);
        // start communication with node
        NodeEffectors_.back().node->Begin();
        // send config messages
        NodeEffectors_.back().node->SetConfigurationMode();
        if (Effector.containsKey("Effectors")) {
          JsonArray &NodeEffectors = Effector["Effectors"];
          for (size_t j=0; j < NodeEffectors.size(); j++) {
            JsonObject &NodeEffector = NodeEffectors[j];
            if (NodeEffector["Type"] == "Motor") {
              NodeEffectors_.back().Types.push_back(kMotor);
              if (NodeEffector.containsKey("Safed-Command")) {
                NodeEffectors_.back().SafedCommands.push_back(NodeEffector["Safed-Command"]);
              }
            }
            if (NodeEffector["Type"] == "Pwm") {
              NodeEffectors_.back().Types.push_back(kPwm);
            }
            if (NodeEffector["Type"] == "Sbus") {
              NodeEffectors_.back().Types.push_back(kSbus);
            }
            if (NodeEffector.containsKey("Input")) {
              NodeEffectors_.back().Inputs.push_back(DefinitionTreePtr->GetValuePtr<float*>(NodeEffector.get<String>("Input").c_str()));
            }
            NodeEffector.printTo(buffer.data(),buffer.size());
            String ConfigString = String("{\"Effectors\":[") + buffer.data() + String("]}");
            NodeEffectors_.back().node->Configure(ConfigString);
          }
        }
      }
    }
  } else {
    while(1){
      Serial.println("ERROR: Effector Configuration failed to parse.");
      Serial.println(JsonString);
    }
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
  std::vector<float> EffectorCommands_;
  size_t EffectorIndex = 0;
  for (size_t i=0; i < Effectors_.size(); i++) {
        if (Effectors_[i].Type != kMotor) {
      Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,EffectorCommands_[EffectorIndex]);
    } else {
      if (ThrottleSafed) {
        Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,Effectors_[i].SafedCommand);
      } else {
        Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,EffectorCommands_[EffectorIndex]);
      }
    }
    EffectorIndex++;
  }
  for (size_t i=0; i < NodeEffectors_.size(); i++) {
    std::vector<float> NodeCommands;
    for (size_t j=0; j < NodeEffectors_[i].Inputs.size(); j++) {
      if (NodeEffectors_[i].Types[j] != kMotor) {
        NodeCommands.push_back(EffectorCommands_[EffectorIndex]);
      } else {
        if (ThrottleSafed) {
          NodeCommands.push_back(NodeEffectors_[i].SafedCommands[j]);
        } else {
          NodeCommands.push_back(EffectorCommands_[EffectorIndex]);
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
      if (ThrottleSafed) {
        Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,Effectors_[i].SafedCommand);
      } else {
        Effectors_[i].Output = PolyVal(Effectors_[i].Calibration,*Effectors_[i].Input);
      }
    }
  }
  for (size_t i=0; i < NodeEffectors_.size(); i++) {
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
