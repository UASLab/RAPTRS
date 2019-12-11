/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
*/

#pragma once

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>

const char InclinePort[] = "/dev/ttyUSB0";
const speed_t InclineBaud = B115200;

struct InclineData {
  double Angle_deg;     // Inclinometer Angle
};

class Incline {
  public:
    Incline();
    void GetAngle(InclineData *InclineDataPtr);
    void SetDamping();
  private:
    int InclineFileDesc_;
    void OpenPort();
};
