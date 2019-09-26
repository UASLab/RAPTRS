/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#pragma once

#include "hardware-defs.h"
#include "configuration.h"

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

class AircraftEffectors {
  public:
    void Configure(const rapidjson::Value& Config);
    std::vector<float> Run();
    bool Configured();
    std::vector<std::string> GetKeys();
  private:
    std::string RootPath_ = "/Effectors";
    bool Configured_ = false;
    std::vector<std::string> EffKeyVec_;
    std::vector<ElementPtr> EffNodeVec_;
};
