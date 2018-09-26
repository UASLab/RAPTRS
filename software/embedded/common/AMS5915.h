/*
AMS5915.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

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

#ifndef AMS5915_h
#define AMS5915_h

#include "i2c_t3.h"
#include "Arduino.h"

class AMS5915{
  public:
    enum Transducer
    {
      AMS5915_0005_D,
      AMS5915_0010_D,
      AMS5915_0005_D_B,
      AMS5915_0010_D_B,
      AMS5915_0020_D,
      AMS5915_0050_D,
      AMS5915_0100_D,
      AMS5915_0020_D_B,
      AMS5915_0050_D_B,
      AMS5915_0100_D_B,
      AMS5915_0200_D,
      AMS5915_0350_D,
      AMS5915_1000_D,
      AMS5915_2000_D,
      AMS5915_4000_D,
      AMS5915_7000_D,
      AMS5915_10000_D,
      AMS5915_0200_D_B,
      AMS5915_0350_D_B,
      AMS5915_1000_D_B,
      AMS5915_1000_A,
      AMS5915_1200_B
    };
    AMS5915(i2c_t3 &bus,uint8_t address,Transducer type);
    int begin();
    int readSensor();
    float getPressure_Pa();
    float getTemperature_C();
  private:
    // struct to hold sensor data
    struct Data {
      float Pressure_Pa;
      float Temp_C;
    };
    Data _data;
    // I2C bus
    i2c_t3 *_bus;
    // i2c timeout, us
    const uint32_t I2cTimeout_us = 200;
    // sensor address
    uint8_t _address;
    // transducer type
    Transducer _type;
    // buffer for I2C data
    uint8_t _buffer[4];
    // number of bytes received from I2C
    size_t _numBytes;
    // maximum number of attempts to talk to sensor
    const size_t _maxAttempts = 10;
    // track success of reading from sensor
    int _status;
    // pressure digital output, counts
    uint16_t _pressureCounts;
    // temperature digital output, counts
    uint16_t _temperatureCounts;
    // min and max pressure, millibar
    int _pMin;
    int _pMax;
    // i2c bus frequency
    const uint32_t _i2cRate = 400000;
    // conversion millibar to PA
    const float _mBar2Pa = 100.0f;
    // digital output at minimum pressure
    const int _digOutPmin = 1638;
    // digital output at maximum pressure
    const int _digOutPmax = 14745;
  	// min and max pressures, millibar
  	const int AMS5915_0005_D_P_MIN = 0;
  	const int AMS5915_0005_D_P_MAX = 5;
  	const int AMS5915_0010_D_P_MIN = 0;
  	const int AMS5915_0010_D_P_MAX = 10;
  	const int AMS5915_0005_D_B_P_MIN = -5;
  	const int AMS5915_0005_D_B_P_MAX = 5;
  	const int AMS5915_0010_D_B_P_MIN = -10;
  	const int AMS5915_0010_D_B_P_MAX = 10;
  	const int AMS5915_0020_D_P_MIN = 0;
  	const int AMS5915_0020_D_P_MAX = 20;
  	const int AMS5915_0050_D_P_MIN = 0;
  	const int AMS5915_0050_D_P_MAX = 50;
  	const int AMS5915_0100_D_P_MIN = 0;
  	const int AMS5915_0100_D_P_MAX = 100;
  	const int AMS5915_0020_D_B_P_MIN = -20;
  	const int AMS5915_0020_D_B_P_MAX = 20;
  	const int AMS5915_0050_D_B_P_MIN = -50;
  	const int AMS5915_0050_D_B_P_MAX = 50;
  	const int AMS5915_0100_D_B_P_MIN = -100;
  	const int AMS5915_0100_D_B_P_MAX = 100;
  	const int AMS5915_0200_D_P_MIN = 0;
  	const int AMS5915_0200_D_P_MAX = 200;
  	const int AMS5915_0350_D_P_MIN = 0;
  	const int AMS5915_0350_D_P_MAX = 350;
  	const int AMS5915_1000_D_P_MIN = 0;
  	const int AMS5915_1000_D_P_MAX = 1000;
  	const int AMS5915_2000_D_P_MIN = 0;
  	const int AMS5915_2000_D_P_MAX = 2000;
  	const int AMS5915_4000_D_P_MIN = 0;
  	const int AMS5915_4000_D_P_MAX = 4000;
  	const int AMS5915_7000_D_P_MIN = 0;
  	const int AMS5915_7000_D_P_MAX = 7000;
  	const int AMS5915_10000_D_P_MIN = 0;
  	const int AMS5915_10000_D_P_MAX = 10000;
  	const int AMS5915_0200_D_B_P_MIN = -200;
  	const int AMS5915_0200_D_B_P_MAX = 200;
  	const int AMS5915_0350_D_B_P_MIN = -350;
  	const int AMS5915_0350_D_B_P_MAX = 350;
  	const int AMS5915_1000_D_B_P_MIN = -1000;
  	const int AMS5915_1000_D_B_P_MAX = 1000;
  	const int AMS5915_1000_A_P_MIN = 0;
  	const int AMS5915_1000_A_P_MAX = 1000;
  	const int AMS5915_1200_B_P_MIN = 700;
  	const int AMS5915_1200_B_P_MAX = 1200;
    void getTransducer();
    int readBytes(uint16_t* pressureCounts, uint16_t* temperatureCounts);
};

#endif
