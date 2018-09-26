/*
control.hxx
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

#ifndef CONTROL_HXX_
#define CONTROL_HXX_

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "generic-function.hxx"
#include "general-functions.hxx"
#include "control-functions.hxx"
#include "allocation-functions.hxx"
#include "filter-functions.hxx"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <Eigen/Dense>
#include <memory>

/* Class to manage control laws
Example JSON configuration:
{ 
  "Control": {
    "Fmu": "FmuGroup",
    "Soc": ["SocGroup1","SocGroup2",...],
    "FmuGroup": [
      { "Level": "1",
        "Components": [
          { "Type": "Gain",
            "Input": 
            "Output":
            "Gain":
          }
        ]
      }
    ]
    "SocGroup1": [
      { "Level": "1",
        "Components": [
          { "Type": "Gain",
            "Input": 
            "Output":
            "Gain":
          }
        ]
      }
    ]
  }
}

Where:
   * FMU is a vector of control law group names for the Flight Management Unit to use.
   * SOC is a vector of control law group names for the SOC to use.
   * Control law goups are named as arrays. Each array contains an object for each control
     law level starting with the outermost level and working to the innermost. Each level
     is named for easy reference by the excitation system. Components for each control
     level are objects configuring the specific control law.
*/

class ControlLaws {
  public:
    void Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr);
    void SetEngagedController(std::string ControlGroupName);
    void SetArmedController(std::string ControlGroupName);
    size_t ActiveControlLevels();
    std::string GetActiveLevel(size_t ControlLevel);
    void RunEngaged(size_t ControlLevel);
    void RunArmed();
  private:
    std::string RootPath_ = "/Control";
    std::string EngagedGroup_ = "Fmu";
    std::string ArmedGroup_ = "Fmu";
    std::map<std::string,std::vector<std::vector<std::shared_ptr<GenericFunction>>>> SocControlGroups_;
    std::vector<std::string> SocGroupKeys_;
    std::map<std::string,std::vector<std::string>> SocLevelNames_;
    std::map<std::string,std::vector<std::vector<std::string>>> SocDataKeys_;
    std::map<std::string,std::variant<uint64_t,uint32_t,uint16_t,uint8_t,int64_t,int32_t,int16_t,int8_t,float, double>> OutputData_;
    std::map<std::string,std::map<std::string,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*,double*>>> SocDataPtr_;
};

#endif
