/*
sensor-processing.hxx
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

#pragma once

#include "hardware-defs.h"
#include "configuration.h"
#include "generic-function.h"
#include "general-functions.h"
#include "filter-functions.h"
#include "airdata-functions.h"
#include "ins-functions.h"
#include "power.h"

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
#include <memory>

/*
  "Sensor-Processing": {
    "Fmu": "System1",
    "Baseline": "System1",
    "Test": ["System1"],

    "Def": {
      "System1": [
        {Component1},
        {Component2},
        {Component3},
        {Component4}
      ]
    }
  }


  Where:
     * FMU is a System name coorsponding to a SensorProcessing System for the Flight Management Unit to use.
     * Baseline is a System name coorsponding to a SensorProcessing System to run as the Baseline system.
        The Baseline system always runs in Armed or Engaged mode.
     * Test is a vector of System names coorsponding to SensorProcessing Systems to run as the Test systems.
        The Baseline system will be excluded from this group if included.
        Test System can run in any mode, but only one system can be Armed or Engaged at a time.

*/

// Wrapper to contain all the Components defined in a SensorProc Definition
class SystemWrapper {
  public:
     void Configure (std::string SystemName, const rapidjson::Value& System);
     void Initialize ();
     bool Initialized ();
     void Run (GenericFunction::Mode mode);
   private:
     std::vector<std::shared_ptr<GenericFunction>> ComponentVec_;
};

class SensorProcessing {
  public:
    void Configure (const rapidjson::Value& Config);
    bool Initialized ();
    void RunBaseline (GenericFunction::Mode mode);
    void SetTest (std::string TestSel);
    std::string GetTest ();
    void RunTest (GenericFunction::Mode mode);
  private:
    std::string RootPath_ = "/Sensor-Processing";
    std::string BaselinePath_, TestPath_;

    bool InitializedLatch_ = false;

    typedef std::shared_ptr<SystemWrapper> SystemWrapperPtr ;

    SystemWrapperPtr BaselinePtr_;

    std::vector<std::string> SystemDefKeys_;
    std::map<std::string, SystemWrapperPtr> SystemMap_;

    std::string TestSel_;

    std::string FmuGroup_;
    std::string BaselineGroup_;
    std::vector<std::string> TestGroups_;

    std::map<std::string, std::vector<std::string>> TestKeys;

    std::map<std::string, ElementPtr> RootNodes;
    std::map<std::string, ElementPtr> BaselineNodes;
    std::map<std::string, map<std::string, ElementPtr> > TestNodes;
};
