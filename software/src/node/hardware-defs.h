/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#ifndef HARDWARE_DEFS_H_
#define HARDWARE_DEFS_H_

#include "i2c_t3.h"
#include "Arduino.h"

// Node Software Version
const String SoftwareVersion = "0.10.1 - mAEWing2";

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
