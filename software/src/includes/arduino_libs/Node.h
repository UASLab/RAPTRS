/*
comms.h
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


#ifndef BFSSENSORCOMMS_H_
#define BFSSENSORCOMMS_H_

#include "Vector.h"
#include "Eigen.h"
#include "i2c_t3.h"
#include "Arduino.h"

class Node {
  public:
    Node(i2c_t3& bus,uint8_t addr,uint32_t rate);
    void Begin();
    void Configure(String ConfigString);
    void Configure(uint8_t id, std::vector<uint8_t> *Payload);
    void SetRunMode();
    void SetConfigurationMode();
    bool ReadSensorData();
    uint8_t GetNumberPwmVoltageSensor();
    uint8_t GetNumberSbusVoltageSensor();
    uint8_t GetNumberMpu9250Sensor();
    uint8_t GetNumberBme280Sensor();
    uint8_t GetNumberuBloxSensor();
    uint8_t GetNumberSwiftSensor();
    uint8_t GetNumberAms5915Sensor();
    uint8_t GetNumberSbusSensor();
    uint8_t GetNumberAnalogSensor();
    void GetSensorDataBuffer(std::vector<uint8_t> *SensorDataBuffer);
    void SendEffectorCommand(std::vector<float> Commands);
  private:
    enum Message {
      ModeCommand,
      Configuration,
      SensorMetaData,
      SensorData,
      EffectorCommand
    };
    enum Mode {
      ConfigurationMode,
      RunMode
    };
    struct Mpu9250SensorData {
      int ReadStatus = -1;                                                          // positive if a good read or negative if not
      Eigen::Matrix<float,3,1>Accel_mss = Eigen::Matrix<float,3,1>::Zero();         // x,y,z accelerometers, m/s/s
      Eigen::Matrix<float,3,1>Gyro_rads = Eigen::Matrix<float,3,1>::Zero();         // x,y,z gyros, rad/s
      Eigen::Matrix<float,3,1>Mag_uT = Eigen::Matrix<float,3,1>::Zero();            // x,y,z magnetometers, uT
      float Temperature_C = 0.0f;                                                   // Temperature, C
    };
    struct Bme280SensorData {
      int ReadStatus = -1;                                                          // positive if a good read or negative if not
      float Pressure_Pa = 0.0f;                                                     // Pressure, Pa
      float Temperature_C = 0.0f;                                                   // Temperature, C
      float Humidity_RH = 0.0f;                                                     // Relative humidity
    };
    struct uBloxSensorData {
      bool Fix = false;                                                             // True for 3D fix only
      uint8_t NumberSatellites = 0;                                                 // Number of satellites used in solution
      uint32_t TOW = 0;                                                             // GPS time of the navigation epoch
      uint16_t Year = 0;                                                            // UTC year
      uint8_t Month = 0;                                                            // UTC month
      uint8_t Day = 0;                                                              // UTC day
      uint8_t Hour = 0;                                                             // UTC hour
      uint8_t Min = 0;                                                              // UTC minute
      uint8_t Sec = 0;                                                              // UTC second
      Eigen::Matrix<double,3,1>LLA = Eigen::Matrix<double,3,1>::Zero();             // Latitude (rad), Longitude (rad), Altitude (m)
      Eigen::Matrix<double,3,1>NEDVelocity_ms  = Eigen::Matrix<double,3,1>::Zero(); // NED Velocity, m/s
      Eigen::Matrix<double,3,1>Accuracy  = Eigen::Matrix<double,3,1>::Zero();       // Horizontal (m), vertical (m), and speed (m/s) accuracy estimates
      double pDOP = 0.0;                                                            // Position DOP
    };
    struct Ams5915SensorData {
      int ReadStatus = -1;                                                          // positive if a good read or negative if not
      float Pressure_Pa = 0.0f;                                                     // Pressure, Pa
      float Temperature_C = 0.0f;                                                   // Temperature, C
    };
    struct SwiftSensorData {
      Ams5915SensorData Static;
      Ams5915SensorData Differential;
    };
    struct SbusSensorData {
      float Channels[16] = {0.0f};
      bool FailSafe = false;
      uint64_t LostFrames = 0;
    };
    struct AnalogSensorData {
      float Voltage_V = 0.0f;
      float CalibratedValue = 0.0f;
    };
    struct SensorData {
      std::vector<float> PwmVoltage_V;
      std::vector<float> SbusVoltage_V;
      std::vector<Mpu9250SensorData> Mpu9250;
      std::vector<Bme280SensorData> Bme280;
      std::vector<uBloxSensorData> uBlox;
      std::vector<SwiftSensorData> Swift;
      std::vector<Ams5915SensorData> Ams5915;
      std::vector<SbusSensorData> Sbus;
      std::vector<AnalogSensorData> Analog;
    };
    static const size_t kBufferMaxSize = 4096;
    const uint32_t I2cHeaderTimeout_us = 200;
    const uint32_t I2cDataTimeout_us = 2000;
    i2c_t3 *bus_;
    uint8_t addr_;
    uint32_t rate_;
    uint8_t Buffer_[kBufferMaxSize];
    const uint8_t header_[2] = {0x42,0x46};
    const size_t headerLength_ = 5;
    const size_t checksumLength_ = 2;
    uint8_t RxByte_;
    uint16_t ParserState_ = 0;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];
    const size_t MetaDataLength_ = 8;
    struct SensorData SensorData_;
    std::vector<uint8_t> DataBuffer_;
    void SendMessage(uint8_t message, std::vector<uint8_t> &Payload);
    void BuildMessage(uint8_t message, std::vector<uint8_t> &Payload, std::vector<uint8_t> *TxBuffer);
    bool ReceiveMessage(Message *message,std::vector<uint8_t> *Payload);
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};

#endif
