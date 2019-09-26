/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <termios.h>
#include <string>

class HardwareSerial {
  public:
    HardwareSerial(const std::string &port);
    bool begin(unsigned int baud);
    unsigned int write(unsigned char data);
    unsigned int write(unsigned char *data, unsigned int len);
    int available();
    unsigned char read();
    int read(unsigned char *data, unsigned int len);
    void end();
  private:
    std::string _port;
    int _fd;
    speed_t get_baud(unsigned int baud);
};
