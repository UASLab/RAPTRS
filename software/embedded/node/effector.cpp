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

/* updates the effector configuration from JSON */
void AircraftEffectors::UpdateConfig(const char *JsonString) {
  Serial.print("Updating effector configuration...");
  DynamicJsonBuffer ConfigBuffer;
  JsonArray &Config = ConfigBuffer.parseArray(JsonString);
  sbus_ = new SBUS(kSbusUart);
  sbus_->begin();
  if (Config.success()) {
    for (size_t i=0; i < Config.size(); i++) {
      JsonObject& Effector = Config[i];
      Data Temp;
      Effectors_.push_back(Temp);
      if ((Effector["Type"] == "Pwm")||(Effector["Type"] == "Motor")) {
        Effectors_.back().Type = kPwm;
      }
      if (Effector["Type"] == "Sbus") {
        Effectors_.back().Type = kSbus;
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
      analogWriteResolution(kPwmResolution);
      analogWriteFrequency(kPwmPins[Effectors_.back().Channel],kPwmFrequency);
      config_.Resolution = powf(2,kPwmResolution) - 1.0f;
      config_.Period = 1.0f/kPwmFrequency * 1000000.0f;
    }
  } else {
    while(1){
      Serial.println("ERROR: Effector Configuration failed to parse.");
      Serial.println(JsonString);
    }
  }
  Serial.println("done!");
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

/* frees the resources used */
void AircraftEffectors::End() {
  Effectors_.clear();
  EffectorCommands_.clear();
  delete sbus_;
  kSbusUart.end();
}
