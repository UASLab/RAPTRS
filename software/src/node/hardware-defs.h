/*
hardware-defs.h
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

#ifndef HARDWARE_DEFS_H_
#define HARDWARE_DEFS_H_

#include "i2c_t3.h"
#include "Arduino.h"

// Node Software Version
const String SoftwareVersion = "0.8.3 - mAEWing2";

// Debug port
const uint32_t kDebugBaud = 115200;                           // Baudrate for status and debug messages

// BFS port
static i2c_t3 &kBfsPort = Wire;                               // I2C port used for communicating with FMU
const i2c_pins kBfsPins = I2C_PINS_18_19;                     // I2C pins used for communicating with FMU
const uint32_t kBfsRate = 3000000;                            // I2C rate used for communicating with FMU
// const uint32_t kBfsRate = 6000000;                            // I2C rate used for communicating with FMU

// Buffers
const size_t kEepromMaxSize = 4096;                           // Max EEPROM size on board
const size_t kUartBufferMaxSize = 4096;                       // Max size for UART buffers

// Sensors
const uint8_t kSyncDataCollectionIntPin = 11;                 // Sync data collection interrupt pin
const uint8_t kSyncEffectorIntPin = 12;                       // Sync effector output interrupt pin
static HardwareSerial &kSbusUart = Serial4;                   // Serial port used for SBUS receive and servo command
const uint8_t kAnalogReadResolution = 16;                     // Resolution used for reading analog
const uint8_t kAnalogPins[8] = {21,20,17,15,14,A22,A21,39};   // Array of pins for analog measurement
const uint8_t kSbusVoltagePin = 22;                           // Pin used for measuring SBUS servo voltage
const uint8_t kPwmVoltagePin = 16;                            // Pin used for measuring PWM servo voltage
const float kEffectorVoltageScale = 3.0f;                     // Scale factor for converting measured to input servo voltage

// Effectors
const uint8_t kPwmPins[8] = {23,2,5,6,7,8,35,36};             // Array of pins for PWM output
const float kPwmFrequency = 50;                               // PWM frequency, Hz
const float kPwmResolution = 16;                              // PWM resolution, bits

#endif
