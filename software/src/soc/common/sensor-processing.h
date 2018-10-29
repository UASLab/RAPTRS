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

#ifndef SENSOR_PROCESSING_HXX_
#define SENSOR_PROCESSING_HXX_

#include "hardware-defs.h"
#include "definition-tree2.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "generic-function.h"
#include "general-functions.h"
#include "flow-control-functions.h"
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
#include <Eigen/Dense>
#include <memory>

class SensorProcessing {
  public:
    void Configure(const rapidjson::Value& Config);
    bool Configured();
    bool Initialized();
    void SetEngagedSensorProcessing(std::string EngagedSensorProcessing);
    void Run();
  private:
    std::string RootPath_ = "/Sensor-Processing";
    bool Configured_ = false;
    bool InitializedLatch_ = false;
    std::string EngagedGroup_ = "Baseline";
    std::vector<std::shared_ptr<GenericFunction>> BaselineSensorProcessing_;
    std::map<std::string,std::vector<std::shared_ptr<GenericFunction>>> ResearchSensorProcessingGroups_;
    std::vector<std::string> BaselineDataKeys_;
    std::vector<std::string> ResearchGroupKeys_;
    std::map<std::string,std::vector<std::string>> ResearchDataKeys_;
    map<string, ElementPtr> OutputDataPtr_;
    map<string, ElementPtr> BaselineDataPtr_;
    map<string, map<string, ElementPtr> > ResearchDataPtr_;
};

#endif
