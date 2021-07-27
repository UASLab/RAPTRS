/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#ifndef SENSORS_H_
#define SENSORS_H_

#include "AMS5915.h"
#include "Adafruit_ADS1X15.h"
#include "HX711.h"
#include "BME280.h"
#include "MPU9250.h"
#include "SBUS.h"
#include "UBLOX.h"
#include "Node.h"
#include "EEPROM.h"
#include "Eigen.h"
#include "Vector.h"
#include "utils.h"
#include "hardware-defs.h"
#include "definition-tree.h"
#include "Arduino.h"

#include "fmu_messages.h"

/* class for the fmu Time */
class TimeSensor {
  public:
    // Data
    uint64_t Time_us = 0;
    int ReadSensor();
    void UpdateMessage(message::data_time_t *msg);
};

/* class for the fmu integrated MPU-9250 */
class InternalMpu9250Sensor {
  public:
    struct Config {
      Eigen::Matrix<float,3,3>Rotation = Eigen::Matrix<float,3,3>::Identity();      // 3x3 rotation matrix
      Eigen::Matrix<float,3,3>Orientation = Eigen::Matrix<float,3,3>::Identity();   // 3x3 rotation matrix
      MPU9250::DlpfBandwidth Bandwidth = MPU9250::DLPF_BANDWIDTH_20HZ;              // MPU-9250 DLPF bandwidth setting
      uint8_t SRD = 19;                                                             // Sample rate divider
    };
    // Data
    float AccelX_mss;        // x,y,z accelerometers, m/s/s
    float AccelY_mss;
    float AccelZ_mss;
    float GyroX_rads;        // x,y,z gyros, rad/s
    float GyroY_rads;
    float GyroZ_rads;
    float MagX_uT;           // x,y,z magnetometers, uT
    float MagY_uT;
    float MagZ_uT;
    float Temperature_C;     // Temperature, C
    bool UpdateConfig(message::config_mpu9250_t *msg,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_mpu9250_t *msg);
    void End();
  private:
    MPU9250 *Mpu_;
    Config config_;
    int8_t status_;
    Eigen::Matrix<float,3,1> Accel_mss_;
    Eigen::Matrix<float,3,1> Gyro_rads_;
    Eigen::Matrix<float,3,1> Mag_uT_;
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
    // Data
    // int8_t ReadStatus = -1;
    // float AccelX_mss = 0;       // x,y,z accelerometers
    // float AccelY_mss = 0;
    // float AccelZ_mss = 0;
    // float GyroX_rads = 0;       // x,y,z gyros
    // float GyroY_rads = 0;
    // float GyroZ_rads = 0;
    // float MagX_uT = 0;       // x,y,z magnetometers
    // float MagY_uT = 0;
    // float MagZ_uT = 0;
    // float Temperature_C = 0.0f;
    bool UpdateConfig(message::config_mpu9250_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
  void UpdateMessage(message::data_mpu9250_short_t *msg);
    void End();
  private:
    MPU9250 *Mpu_;
    Config config_;
    int8_t status_;
    Eigen::Matrix<float,3,1> Accel_mss_;
    Eigen::Matrix<float,3,1> Gyro_rads_;
    Eigen::Matrix<float,3,1> Mag_uT_;
};

/* class for the fmu integrated BME-280 */
class InternalBme280Sensor {
  public:
    struct Config {};
    // Data
    uint8_t status_ = 0;
    float Pressure_Pa = 0.0f;                                                     // Pressure, Pa
    float Temperature_C = 0.0f;                                                   // Temperature, C
    float Humidity_RH = 0.0f;                                                     // Relative humidity
    bool UpdateConfig(std::string Output,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_bme280_t *msg);
    void End();
  private:
    BME280 *Bme_;
    Config config_;
};

/* class for external BME-280 sensors */
class Bme280Sensor {
  public:
    struct Config {
      bool UseSpi = false;
      uint8_t Addr;
      uint8_t CsPin;
    };
    // Data
    int8_t ReadStatus = -1;                                                          // positive if a good read or negative if not
    float Pressure_Pa = 0.0f;                                                     // Pressure, Pa
    float Temperature_C = 0.0f;                                                   // Temperature, C
    float Humidity_RH = 0.0f;                                                     // Relative humidity
    bool UpdateConfig(message::config_bme280_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_bme280_t *msg);
    void End();
  private:
    BME280 *Bme_;
    Config config_;
    int8_t status_;
};

/* class for uBlox sensors */
class uBloxSensor {
  public:
    struct Config {
      uint8_t Uart;                                                                 // UART port
      uint32_t Baud;                                                                // Baudrate
    };
    // Data
    bool Fix;                                 // True for 3D fix only
    uint8_t NumberSatellites;                 // Number of satellites used in solution
    uint32_t TOW;                             // GPS time of the navigation epoch
    uint16_t Year;                            // UTC year
    uint8_t Month;                            // UTC month
    uint8_t Day;                              // UTC day
    uint8_t Hour;                             // UTC hour
    uint8_t Min;                              // UTC minute
    uint8_t Sec;                              // UTC second
    double Latitude_rad;                      // Latitude (rad), Longitude (rad), Altitude (m)
    double Longitude_rad;
    float Altitude_m;
    float NorthVelocity_ms;                   // NED Velocity, m/s
    float EastVelocity_ms;
    float DownVelocity_ms;
    float HorizontalAccuracy_m;               // Accuracy Horizontal (m)
    float VerticalAccuracy_m;                 // Accuracy Vertical (m)
    float VelocityAccuracy_ms;                // Accuracy Speed (m/s)
    float pDOP;                               // Position DOP
    bool UpdateConfig(message::config_ublox_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_ublox_t *msg);
    void End();
  private:
    UBLOX *ublox_;
    gpsData uBloxData_;
    Config config_;
    const float kD2R = PI/180.0f;
};

/* class for AMS5915 sensors */
class Ams5915Sensor {
  public:
    struct Config {
      uint8_t Addr;                                                                 // I2C address
      AMS5915::Transducer Transducer;                                               // Transducer type
    };
    // Data
    int8_t ReadStatus = -1;                                                          // positive if a good read or negative if not
    float Pressure_Pa = 0.0f;                                                     // Pressure, Pa
    float Temperature_C = 0.0f;                                                   // Temperature, C
    bool UpdateConfig(message::config_ams5915_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_ams5915_t *msg);
    void End();
  private:
    AMS5915 *ams_;
    Config config_;
    int8_t status_;
};

/* class for Load Cell HX711 sensors */
class HX711Sensor {
  public:
    struct Config {
      uint8_t Addr;          // I2C address
      HX711::Transducer Transducer; // Transducer type
    };
    // Data
    int8_t ReadStatus = -1; // positive if a good read or negative if not
    float Load = 0.0f;      // Load, grams
    bool UpdateConfig(message::config_hx711_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_ams5915_t *msg);
    void End();
  private:
    HX711 *hx711_;
    Config config_;
    int8_t status_;
};

/* class for Swift sensors */
class SwiftSensor {
  public:
    struct Config {
      Ams5915Sensor::Config Static;
      Ams5915Sensor::Config Differential;
    };
    // Data
    bool UpdateConfig(message::config_swift_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_swift_t *msg);
    void End();
  private:
    Ams5915Sensor StaticAms, DiffAms;
    Config config_;
    int8_t StaticStatus_;
    int8_t DiffStatus_;
};

/* class for SBUS sensors */
class SbusSensor {
  public:
    struct Config {};
    // Data
    //float Channels[16] = {0.0f};
    //bool FailSafe = false;
    //uint64_t LostFrames = 0;
  bool UpdateConfig(std::string output, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_sbus_t *msg);
    void End();
  private:
    float channels_[16];
    uint8_t failsafe_;
    uint64_t lostframes_;
    SBUS *sbus_;
    Config config_;
};

/* class for analog sensors */
class AnalogSensor {
  public:
    struct Config {
      uint8_t Channel;
      uint8_t DirectPin = 0; // allow specifying the actual pin instead of a channel
      std::vector<float> Calibration;
    };
    // Data
    float Voltage_V = 0.0f;
    float CalibratedValue = 0.0f;
    bool UpdateConfig(message::config_analog_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_analog_t *msg);
    void End();
  private:
    Config config_;
};

/* class for nodes */
class SensorNodes {
  public:
    struct Config {
      uint8_t BfsAddr;
    };
    SensorNodes(uint8_t address);
    bool UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload, std::string RootPath, DefinitionTree *DefinitionTreePtr);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    uint8_t GetBfsAddr() { return config_.BfsAddr; }
    void Begin(/*Data *DataPtr*/);
    int ReadSensor();
    int GetMessage(std::vector<uint8_t> *NodeMessageBuffer);
    void End();
  private:
    Config config_;
    // Data data_;
    class Node *node_ = NULL;
};

/* aircraft sensors class wrapping the individual sensor classes */
class AircraftSensors {
  public:
    struct Classes {
      TimeSensor Time;
      //AnalogSensor InputVoltage;
      //AnalogSensor RegulatedVoltage;
      //AnalogSensor PwmVoltage;
      //AnalogSensor SbusVoltage;
      InternalMpu9250Sensor InternalMpu9250;
      InternalBme280Sensor InternalBme280;
      std::vector<Mpu9250Sensor> Mpu9250;
      std::vector<Bme280Sensor> Bme280;
      std::vector<uBloxSensor> uBlox;
      std::vector<SwiftSensor> Swift;
      std::vector<Ams5915Sensor> Ams5915;
      std::vector<SbusSensor> Sbus;
      std::vector<AnalogSensor> Analog;
      std::vector<SensorNodes> Nodes;
    };
    Classes classes;
    bool AcquireInternalMpu9250Data = false;
    bool AcquireInternalBme280Data = false;
    bool AcquireTimeData = false;
    bool AcquireInputVoltageData = false;
    bool AcquireRegulatedVoltageData = false;
    bool AcquirePwmVoltageData = false;
    bool AcquireSbusVoltageData = false;
    bool UpdateConfig(uint8_t id, uint8_t address, std::vector<uint8_t> *Payload, DefinitionTree *DefinitionTreePtr);
    void Begin();
    void ReadSyncSensors();
    void ReadAsyncSensors();
    void End();
  private:
    std::string RootPath_ = "/Sensors";
    bool ResetI2cBus_;
    bool ResetBfsBus_;
    size_t SerializedDataMetadataSize = 10;
};

#endif
