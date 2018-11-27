/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2018 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "HardwareSerial.h"

HardwareSerial::HardwareSerial(const std::string &port)
{
  _port = port;
}

bool HardwareSerial::begin(unsigned int baud)
{
  struct termios options;
  speed_t _baud;
  if ((_fd = open(_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
    return false;
  }
  if ((_baud = get_baud(baud)) < 0) {
    return false;
  }
  if (tcgetattr(_fd,&options) < 0) {
    return false;
  }
  options.c_cflag = _baud | CS8 | CREAD | CLOCAL;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;
  if (tcflush(_fd,TCIFLUSH) < 0) {
    return false;
  }
  if (tcsetattr(_fd,TCSANOW,&options) < 0) {
    return false;
  }
  if (fcntl(_fd,F_SETFL,O_NONBLOCK) < 0) {
    return false;
  }
  return true;
}

unsigned int HardwareSerial::write(unsigned char data)
{
  return ::write(_fd,&data,sizeof(unsigned char));
}

unsigned int HardwareSerial::write(unsigned char *data, unsigned int len)
{
  return ::write(_fd,&data[0],len*sizeof(unsigned char));
}

int HardwareSerial::available()
{
  int bytes_avail;
  ioctl(_fd, FIONREAD, &bytes_avail);
  return bytes_avail;
}

unsigned char HardwareSerial::read()
{
  unsigned char buf;
  ::read(_fd,&buf,1);
  return buf;
}

int HardwareSerial::read(unsigned char *data, unsigned int len)
{
  return ::read(_fd,data,len);
}

void HardwareSerial::end()
{
  close(_fd);
}

speed_t HardwareSerial::get_baud(unsigned int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    default: 
      return -1;
  }
}
