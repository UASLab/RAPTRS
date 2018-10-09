

#ifndef INCLINE_HXX_
#define INCLINE_HXX_

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>

const char InclinePort[] = "/dev/ttyO1";
const speed_t InclineBaud = B115200;

struct InclineData {
  double Angle_deg;     // Inclinometer Angle
};

class Incline {
  public:
    Incline();
    bool GetAngle(InclineData *InclineDataPtr);
    bool SetDamping();
  private:
    int InclineFileDesc_;
    void OpenPort();
};

#endif
