/*
excitation.hxx
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

#ifndef EXCITATION_HXX_
#define EXCITATION_HXX_

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "generic-function.hxx"
#include "general-functions.hxx"
#include "excitation-waveforms.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
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

/* Class to manage excitations
Example JSON configuration:
{ 
  "Excitation": {
    "Time": "",
    "Groups": [
      { "Name": "GroupName"
        "Components": [
          { "Level": "1",
            "Components": [
              {"Waveform": "0_5-Sec-Pulse","Signal": "","Start-Time": 1,,"Amplitude": 0.2}
            ]
          }
        ]
      }
    ]
    "0_5-Sec-Pulse": {
      "Type": "Pulse"
      "Duration": 5
    }
  }
}

Where:
   * Time is the full path to the current system time in us
   * Groups is a vector of excitation groups, each group contains:
      * Name, the name of the group used to identify it in the test points
      * Components, the group components, organized into levels corresponding to
        the control law levels and definining a vector of waveforms that will be run
         * Each waveform is defined by name, the signal it's modifying (i.e. "/Control/Pitch_Cmd"),
           the start time in seconds and the amplitude
   * Each waveform is defined by name, the type of waveform, and any other information needed by the specific waveform type

*/

class ExcitationSystem {
  public:
    void Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr);
    void SetEngagedExcitation(std::string ExcitationGroupName);
    void RunEngaged(std::string ControlLevel);
    void RunArmed();
  private:
    std::string RootPath_ = "/Excitation";
    std::string EngagedGroup_ = "None";
    std::vector<std::string> ExcitationGroupKeys_;
    std::vector<std::vector<std::string>> ExcitationGroupLevels_;
    std::vector<std::vector<std::vector<std::shared_ptr<GenericFunction>>>> ExcitationGroups_;
};

#endif
