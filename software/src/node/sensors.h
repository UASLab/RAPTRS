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

#include "AMS5915.h"
#include "BME280.h"
#include "MPU9250.h"
#include "SBUS.h"
#include "UBLOX.h"
#include "utils.h"
#include "hardware-defs.h"
#include "Vector.h"
#include "Eigen.h"
#include "EEPROM.h"
#include "i2c_t3.h"
#include "SPI.h"
#include "Arduino.h"

#include "fmu_messages.h"

/* class for external MPU-9250 sensors */
class Mpu9250Sensor {
  public:
    struct Config {
      bool UseSpi = false;
      uint8_t I2c = 1;
      uint8_t Addr;
      uint8_t Spi = 0;
      uint8_t CsPin;
      uint8_t MosiPin = 7;
      uint8_t MisoPin = 8;
      uint8_t SckPin = 14;
      Eigen::Matrix<float,3,3>Rotation = Eigen::Matrix<float,3,3>::Identity();      // 3x3 rotation matrix
      MPU9250::DlpfBandwidth Bandwidth = MPU9250::DLPF_BANDWIDTH_20HZ;              // MPU-9250 DLPF bandwidth setting
      uint8_t SRD = 0;                                                              // Sample rate divider
    };
    void UpdateConfig(message::config_mpu9250_t *msg);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_mpu9250_t *msg);
    void End();
  private:
    MPU9250 *Mpu_;
    Config config_;
    SPIClass *_spi;
    i2c_t3 *_i2c;
    int8_t status_;
    Eigen::Matrix<float,3,1> Accel_mss_;
    Eigen::Matrix<float,3,1> Gyro_rads_;
    Eigen::Matrix<float,3,1> Mag_uT_;
};

/* class for external BME-280 sensors */
class Bme280Sensor {
  public:
    struct Config {
      bool UseSpi = false;
      uint8_t I2c = 1;
      uint8_t Addr;
      uint8_t Spi = 0;
      uint8_t CsPin;
      uint8_t MosiPin = 7;
      uint8_t MisoPin = 8;
      uint8_t SckPin = 14;
    };
    void UpdateConfig(message::config_bme280_t *msg);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_bme280_t *msg);
    void End();
  private:
    BME280 *Bme_;
    Config config_;
    SPIClass *_spi;
    i2c_t3 *_i2c;
    int8_t status_;
};

/* class for uBlox sensors */
class uBloxSensor {
  public:
    struct Config {
      uint8_t Uart;                                                                 // UART port
      uint32_t Baud;                                                                // Baudrate
    };
    void UpdateConfig(message::config_ublox_t *msg);
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
      uint8_t I2c = 1;
      uint8_t Addr;                                                              // I2C address
      AMS5915::Transducer Transducer;                                               // Transducer type
    };
    void UpdateConfig(message::config_ams5915_t *msg);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_ams5915_t *msg);
    void End();
  private:
    AMS5915 *ams_;
    Config config_;
    i2c_t3 *_i2c;
    int8_t status_;
};

/* class for Swift sensors */
class SwiftSensor {
  public:
    struct Config {
      Ams5915Sensor::Config Static;
      Ams5915Sensor::Config Differential;
    };
    void UpdateConfig(message::config_swift_t *msg);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_swift_t *msg);
    void End();
  private:
    Ams5915Sensor StaticAms, DiffAms;
    Config config_;
    // Data data_;
    int8_t StaticStatus_;
    int8_t DiffStatus_;
};

/* class for SBUS sensors */
class SbusSensor {
  public:
    struct Config {};
    void UpdateConfig();
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
      std::vector<float> Calibration;
    };
    void UpdateConfig(message::config_analog_t *msg);
    void SetConfig(const Config &ConfigRef);
    void GetConfig(Config *ConfigPtr);
    void Begin();
    int ReadSensor();
    void UpdateMessage(message::data_analog_t *msg);
    void End();
  private:
    Config config_;
    float Voltage_V_;
};

/* aircraft sensors class wrapping the individual sensor classes */
class AircraftSensors {
  public:
    struct Classes {
      std::vector<Mpu9250Sensor> Mpu9250;
      std::vector<Bme280Sensor> Bme280;
      std::vector<uBloxSensor> uBlox;
      std::vector<SwiftSensor> Swift;
      std::vector<Ams5915Sensor> Ams5915;
      std::vector<SbusSensor> Sbus;
      std::vector<AnalogSensor> Analog;
      AnalogSensor PwmVoltageSensor;
      AnalogSensor SbusVoltageSensor;
    };
    bool UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload);
    void Begin();
    void ReadSyncSensors();
    void ReadAsyncSensors();
    void MakeMegaMessage(std::vector<uint8_t> *Buffer);
    void End();
  private:
    Classes classes_;
  private:
    bool ResetI2cBus1_;
    bool ResetI2cBus2_;
    bool AcquirePwmVoltageData_ = false;
    bool AcquireSbusVoltageData_ = false;
};

#endif
