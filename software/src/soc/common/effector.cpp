/*
effector.cc
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

/*
Method to configure effectors and register inputs with definition tree. A typical JSON configuration would be:

"Effectors": [
  { "Type": "Sbus",
    "Input": "/Control/Elevator",
    "Channel": 2,
    "Calibration": [40.975,991.5]},
    "Safed-Command": 
]

Where:
   * Type can be Sbus, Pwm, or Motor specifying whether the effector is a SBUS or PWM servo or a motor. 
     Motors are considered PWM servos, but include throttle safety logic.
   * Input specifies the path name to the input command.
   * Channel is the channel number of the effector.
   * Calibration is a vector of polynomial coefficients listed in descending order to convert the angle
     or power command to an SBUS (172-1811) or PWM (1000-2000 us) output value.
   * Safed command is an optional value specifying the command to output for a motor when the throttle is
     safed. This is the input command to the calibration, not the value sent to the motor (i.e. a value of
     -1 instead of 1000 us).  
*/

/*
   Note: effector.cpp needs to know about physical hardware layout
   (i.e. where actuators are connected down stream and to which nodes.
   Preferably this knowledge should live inside fmu.cpp.  The flight
   code shouldn't need to care about the physical layout of the
   effectors.

   This code does definition tree map lookups every frame.  This way
   when the /Control/ aliases get remade, this module isn't left
   pointing to the old Element records.
*/

void AircraftEffectors::Configure(const rapidjson::Value& Config) {
  assert(Config.IsArray());
  for (size_t i=0; i < Config.Size(); i++) {
    const rapidjson::Value& Effector = Config[i];
    if (Effector.HasMember("Type")) {
      if (Effector["Type"] == "Node") {
        if (Effector.HasMember("Effectors")) {
          assert(Effector["Effectors"].IsArray());
          for (auto &NodeEffector : Effector["Effectors"].GetArray()) {
            if (NodeEffector.HasMember("Input")) {
              ElementPtr ele =  deftree.getElement(NodeEffector["Input"].GetString());
              if ( ele  ) {
                input_nodes.push_back(ele);
              } else {
                throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Input ")+NodeEffector["Input"].GetString()+std::string(" not found in global data."));
              }
            } else {
              throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Input not specified in configuration."));
            }
          }
        } else {
          throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Node effectors not specified in configuration."));
        }
      } else {
        if (Effector.HasMember("Input")) {
          ElementPtr ele = deftree.getElement(Effector["Input"].GetString());
          if ( ele ) {
            input_nodes.push_back(ele);
          } else {
            throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Input ")+Effector["Input"].GetString()+std::string(" not found in global data."));
          }
        } else {
          throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Input not specified in configuration."));
        }
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Type not specified in configuration."));
    }
  }
  Configured_ = true;
}

/* Run method for effectors, dereferences the inputs and returns a vector of effector commands to send to the FMU */
std::vector<float> AircraftEffectors::Run() {
  std::vector<float> Commands;
  Commands.resize(input_nodes.size());
  for (size_t i=0; i < input_nodes.size(); i++) {
    Commands[i] = input_nodes[i]->getFloat();
  }
  return Commands;
}

/* Returns whether the class has been configured */
bool AircraftEffectors::Configured() {
  return Configured_;
}
