/*
sensors.h
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

#ifndef SENSORS_H_
#define SENSORS_H_

#include "../common/AMS5915.h"
#include "../common/BME280.h"
#include "../common/MPU9250.h"
#include "../common/SBUS.h"
#include "../common/UBLOX.h"
#include "../common/Node.h"
#include "../common/EEPROM.h"
#include "../common/Eigen.h"
#include "../common/Vector.h"
#include "../common/ArduinoJson.h"
#include "utils.h"
#include "hardware-defs.h"
#include "definition-tree.h"
#include "Arduino.h"

/* class for the fmu integrated MPU-9250 */
class InternalMpu9250Sensor {
  public:
    struct Config {
      Eigen::Matrix<float,3,3>Rotation = Eigen::Matrix<float,3,3>::Identity();      // 3x3 rotation matrix
      Eigen::Matrix<float,3,3>Orientation = Eigen::Matrix<float,3,3>::Identity();   // 3x3 rotation matrix
      MPU9250::DlpfBandwidth Bandwidth = MPU9250::DLPF_BANDWIDTH_20HZ;              // MPU-9250 DLPF bandwidth setting
      uint8_t SRD = 19;                                                             // Sample rate divider
    };
    struct Data {
      Eigen::Matrix<float,3,1>Accel_mss = Eigen::Matrix<float,3,1>::Zero();         // x,y,z accelerometers, m/s/s
      Eigen::Matrix<float,3,1>Gyro_rads = Eigen::Matrix<float,3,1>::Zero();         // x,y,z gyros, rad/s
      Eigen::Matrix<float,3,1>Mag_uT = Eigen::Matrix<float,3,1>::Zero();            // x,y,z magnetometers, uT
      float Temperature_C = 0.0f;                                                   // Temperature, C
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    void GetData(Data *DataPtr);
    void End();
  private:
    MPU9250 *Mpu_;
    Config config_;
    Data data_;
};

/* class for external MPU-9250 sensors */
class Mpu9250Sensor {
  public:
    struct Config {
      bool UseSpi = false;
      uint8_t Addr;
      uint8_t CsPin;
      Eigen::Matrix<float,3,3>Rotation = Eigen::Matrix<float,3,3>::Identity();      // 3x3 rotation matrix
      MPU9250::DlpfBandwidth Bandwidth = MPU9250::DLPF_BANDWIDTH_20HZ;              // MPU-9250 DLPF bandwidth setting
      uint8_t SRD = 0;                                                              // Sample rate divider
    };
    struct Data {
      int ReadStatus = -1;                                                          // positive if a good read or negative if not
      Eigen::Matrix<float,3,1>Accel_mss = Eigen::Matrix<float,3,1>::Zero();         // x,y,z accelerometers, m/s/s
      Eigen::Matrix<float,3,1>Gyro_rads = Eigen::Matrix<float,3,1>::Zero();         // x,y,z gyros, rad/s
      Eigen::Matrix<float,3,1>Mag_uT = Eigen::Matrix<float,3,1>::Zero();            // x,y,z magnetometers, uT
      float Temperature_C = 0.0f;                                                   // Temperature, C
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int GetData(Data *DataPtr);
    void End();
  private:
    MPU9250 *Mpu_;
    Config config_;
    Data data_;
    int status_;
};

/* class for the fmu integrated BME-280 */
class InternalBme280Sensor {
  public:
    struct Config {};
    struct Data {
      float Pressure_Pa = 0.0f;                                                     // Pressure, Pa
      float Temperature_C = 0.0f;                                                   // Temperature, C
      float Humidity_RH = 0.0f;                                                     // Relative humidity
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    void GetData(Data *DataPtr);
    void End();
  private:
    BME280 *Bme_;
    Config config_;
    Data data_;
};

/* class for external BME-280 sensors */
class Bme280Sensor {
  public:
    struct Config {
      bool UseSpi = false;
      uint8_t Addr;
      uint8_t CsPin;
    };
    struct Data {
      int ReadStatus = -1;                                                          // positive if a good read or negative if not
      float Pressure_Pa = 0.0f;                                                     // Pressure, Pa
      float Temperature_C = 0.0f;                                                   // Temperature, C
      float Humidity_RH = 0.0f;                                                     // Relative humidity
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int GetData(Data *DataPtr);
    void End();
  private:
    BME280 *Bme_;
    Config config_;
    Data data_;
    int status_;
};

/* class for uBlox sensors */
class uBloxSensor {
  public:
    struct Config {
      uint8_t Uart;                                                                 // UART port
      uint32_t Baud;                                                                // Baudrate
    };
    struct Data {
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
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    void UpdateData();
    void GetData(Data *DataPtr);
    void End();
  private:
    UBLOX *ublox_;
    gpsData uBloxData_;
    Config config_;
    Data data_;
    const float kD2R = PI/180.0f;
};

/* class for AMS5915 sensors */
class Ams5915Sensor {
  public:
    struct Config {
      uint8_t Addr;                                                                 // I2C address
      AMS5915::Transducer Transducer;                                               // Transducer type
    };
    struct Data {
      int ReadStatus = -1;                                                          // positive if a good read or negative if not
      float Pressure_Pa = 0.0f;                                                     // Pressure, Pa
      float Temperature_C = 0.0f;                                                   // Temperature, C
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int GetData(Data *DataPtr);
    void End();
  private:
    AMS5915 *ams_;
    Config config_;
    Data data_;
    int status_;
};

/* class for Swift sensors */
class SwiftSensor {
  public:
    struct Config {
      Ams5915Sensor::Config Static;
      Ams5915Sensor::Config Differential;
    };
    struct Data {
      Ams5915Sensor::Data Static;
      Ams5915Sensor::Data Differential;
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int GetData(Data *DataPtr);
    void End();
  private:
    Ams5915Sensor StaticAms, DiffAms;
    Config config_;
    Data data_;
    int StaticStatus_;
    int DiffStatus_;
};

/* class for SBUS sensors */
class SbusSensor {
  public:
    struct Config {};
    struct Data {
      float Channels[16] = {0.0f};
      bool FailSafe = false;
      uint64_t LostFrames = 0;
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    void UpdateData();
    void GetData(Data *DataPtr);
    void End();
  private:
    float channels_[16];
    uint8_t failsafe_;
    uint64_t lostframes_;
    SBUS *sbus_;
    Config config_;
    Data data_;
};

/* class for analog sensors */
class AnalogSensor {
  public:
    struct Config {
      uint8_t Channel;
      std::vector<float> Calibration;
    };
    struct Data {
      float Voltage_V = 0.0f;
      float CalibratedValue = 0.0f;
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    void GetData(Data *DataPtr);
    void End();
  private:
    Config config_;
    Data data_;
};

/* class for nodes */
class SensorNodes {
  public:
    struct Config {
      uint8_t BfsAddr;
    };
    struct Data {
      std::vector<float> PwmVoltage_V;
      std::vector<float> SbusVoltage_V;
      std::vector<Mpu9250Sensor::Data> Mpu9250;
      std::vector<Bme280Sensor::Data> Bme280;
      std::vector<uBloxSensor::Data> uBlox;
      std::vector<SwiftSensor::Data> Swift;
      std::vector<Ams5915Sensor::Data> Ams5915;
      std::vector<SbusSensor::Data> Sbus;
      std::vector<AnalogSensor::Data> Analog;
    };
    void UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin(Data *DataPtr);
    int GetData(Data *DataPtr);
    void End();
  private:
    Config config_;
    Data data_;
    class Node *node_;
};

/* aircraft sensors class wrapping the individual sensor classes */
class AircraftSensors {
  public:
    struct Classes {
      InternalMpu9250Sensor InternalMpu9250;
      std::vector<InternalBme280Sensor> InternalBme280;
      std::vector<Mpu9250Sensor> Mpu9250;
      std::vector<Bme280Sensor> Bme280;
      std::vector<uBloxSensor> uBlox;
      std::vector<SwiftSensor> Swift;
      std::vector<Ams5915Sensor> Ams5915;
      std::vector<SbusSensor> Sbus;
      std::vector<AnalogSensor> Analog;
      std::vector<SensorNodes> Nodes;
    };
    struct Data {
      std::vector<uint64_t> Time_us;
      std::vector<InternalMpu9250Sensor::Data> InternalMpu9250;
      std::vector<InternalBme280Sensor::Data> InternalBme280;
      std::vector<float> InputVoltage_V;
      std::vector<float> RegulatedVoltage_V;
      std::vector<float> PwmVoltage_V;
      std::vector<float> SbusVoltage_V;
      std::vector<Mpu9250Sensor::Data> Mpu9250;
      std::vector<Bme280Sensor::Data> Bme280;
      std::vector<uBloxSensor::Data> uBlox;
      std::vector<SwiftSensor::Data> Swift;
      std::vector<Ams5915Sensor::Data> Ams5915;
      std::vector<SbusSensor::Data> Sbus;
      std::vector<AnalogSensor::Data> Analog;
      std::vector<SensorNodes::Data> Nodes;
    };
    void UpdateConfig(const char *JsonString,DefinitionTree *DefinitionTreePtr);
    void Begin();
    void ReadSyncSensors();
    void ReadAsyncSensors();
    void GetData(Data *DataPtr);
    void GetDataBuffer(std::vector<uint8_t> *Buffer);
    void End();
  private:
    std::string RootPath_ = "/Sensors";
    Classes classes_;
    Data data_;
    bool ResetI2cBus_;
    bool ResetBfsBus_;
    bool AcquireInternalMpu9250Data_ = false;
    bool AcquireTimeData_ = false;
    bool AcquireInputVoltageData_ = false;
    bool AcquireRegulatedVoltageData_ = false;
    bool AcquirePwmVoltageData_ = false;
    bool AcquireSbusVoltageData_ = false;
    size_t SerializedDataMetadataSize = 10;
};

#endif
