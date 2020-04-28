/*
Copyright (c) 2018 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan, Curt Olson
*/

#pragma once

#include <stdlib.h>
#include <iostream>
#include <string>

#include "configuration.h"

class SimMgr {
  public:
    bool Configure( const rapidjson::Value& Config );

    std::string ModelName;
    std::string FmuPort = "/dev/ttyO4";
    uint32_t FmuBaud = 1500000;
};
