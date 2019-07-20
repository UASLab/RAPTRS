/*
fmu.hxx
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

#ifndef FMU_HXX_
#define FMU_HXX_

#include "hardware-defs.h"
#include "definition-tree2.h"
#include "fmu_messages.h"
#include "SerialLink.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <string>
#include <vector>
using std::vector;

class FlightManagementUnit {
  public:
    enum Message {
      kModeCommand,
      kConfigMesg,
      kSensorData,
      kEffectorCommand
    };
    enum Mode {
      kConfigMode,
      kRunMode
    };
    void Begin();
    void Configure(const rapidjson::Value& Config);
    void SendModeCommand(Mode mode);
    bool ReceiveSensorData(bool publish=true);
    void SendEffectorCommands(std::vector<float> Commands);
  private:
    struct InternalMpu9250SensorData {
      float AccelX_mss;        // x,y,z accelerometers, m/s/s
      float AccelY_mss;
      float AccelZ_mss;
      float GyroX_rads;        // x,y,z gyros, rad/s
      float GyroY_rads;
      float GyroZ_rads;
      float MagX_uT;           // x,y,z magnetometers, uT
      float MagY_uT;
      float MagZ_uT;
      float Temperature_C;                      // Temperature, C
    };
    struct InternalMpu9250SensorNodes {
      ElementPtr ax, ay, az;
      ElementPtr p, q, r;
      ElementPtr hx, hy, hz;
      ElementPtr temp;
    };
    struct InternalBme280SensorData {
      float Pressure_Pa;                        // Pressure, Pa
      float Temperature_C;                      // Temperature, C
      float Humidity_RH;                        // Relative humidity
    };
    struct InternalBme280SensorNodes {
      ElementPtr press;
      ElementPtr temp;
      ElementPtr hum;
    };
    struct Mpu9250SensorData {
      int8_t status;
      int16_t AccelX_ct = 0; // x,y,z accelerometers, m/s/s
      int16_t AccelY_ct = 0;
      int16_t AccelZ_ct = 0;
      int16_t GyroX_ct = 0; // x,y,z gyros, rad/s
      int16_t GyroY_ct = 0;
      int16_t GyroZ_ct = 0;
      // int16_t MagX_ct = 0; // x,y,z magnetometers, uT
      // int16_t MagY_ct = 0;
      // int16_t MagZ_ct = 0;
      // int16_t Temperature_ct = 0.0f; // Temperature, C
    };
    struct Mpu9250SensorNodes {
      ElementPtr status;
      ElementPtr ax, ay, az;
      ElementPtr p, q, r;
      // ElementPtr hx, hy, hz;
      // ElementPtr temp;
    };
    struct Bme280SensorData {
      int8_t status;
      float Pressure_Pa;                        // Pressure, Pa
      float Temperature_C;                      // Temperature, C
      float Humidity_RH;                        // Relative humidity
    };
    struct Bme280SensorNodes {
      ElementPtr status;
      ElementPtr press;
      ElementPtr temp;
      ElementPtr hum;
    };
    struct uBloxSensorData {
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
    };
    struct uBloxSensorNodes {
      ElementPtr fix;
      ElementPtr sats;
      ElementPtr tow;
      ElementPtr year;
      ElementPtr month;
      ElementPtr day;
      ElementPtr hour;
      ElementPtr min;
      ElementPtr sec;
      ElementPtr lat, lon, alt;
      ElementPtr vn, ve, vd;
      ElementPtr horiz_acc, vert_acc, vel_acc;
      ElementPtr pdop;
    };
    struct Ams5915SensorData {
      int8_t status;
      float Pressure_Pa;                        // Pressure, Pa
      float Temperature_C;                      // Temperature, C
    };
    struct Ams5915SensorNodes {
      ElementPtr status;
      ElementPtr press;
      ElementPtr temp;
    };
    struct SwiftSensorData {
      Ams5915SensorData Static;
      Ams5915SensorData Differential;
    };
    struct SwiftSensorNodes {
      Ams5915SensorNodes Static;
      Ams5915SensorNodes Differential;
    };
    struct SbusSensorData {
      float Channels[16];
      bool FailSafe;
      uint64_t LostFrames;
    };
    struct SbusSensorNodes {
      ElementPtr ch[16];
      ElementPtr failsafe;
      ElementPtr lost_frames;
    };
    struct AnalogSensorData {
      // float Voltage_V;
      float CalibratedValue;
    };
    struct AnalogSensorNodes {
      // ElementPtr volt;
      ElementPtr val;
    };
    struct SensorData {
      std::vector<uint64_t> Time_us;
      std::vector<InternalMpu9250SensorData> InternalMpu9250;
      std::vector<InternalBme280SensorData> InternalBme280;
      std::vector<float> InputVoltage_V;
      std::vector<float> RegulatedVoltage_V;
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
    struct SensorNodes {
      vector<ElementPtr> Time_us;
      vector<InternalMpu9250SensorNodes> InternalMpu9250;
      vector<InternalBme280SensorNodes> InternalBme280;
      vector<ElementPtr> input_volts;
      vector<ElementPtr> reg_volts;
      vector<ElementPtr> pwm_volts;
      vector<ElementPtr> sbus_volts;
      vector<Mpu9250SensorNodes> Mpu9250;
      vector<Bme280SensorNodes> Bme280;
      vector<uBloxSensorNodes> uBlox;
      vector<SwiftSensorNodes> Swift;
      vector<Ams5915SensorNodes> Ams5915;
      vector<SbusSensorNodes> Sbus;
      vector<AnalogSensorNodes> Analog;
    };
    const std::string Port_ = FmuPort;
    const std::string RootPath_ = "/Sensors";
    const uint32_t Baud_ = FmuBaud;
    HardwareSerial *_serial;
    SerialLink *_bus;
    message::mode_command_t cmd_msg;
    message::config_ack_t config_ack_msg;
    SensorData SensorData_;
    SensorNodes SensorNodes_;
    // static const
    bool WaitForAck(uint8_t id, uint8_t subid, float timeout_millis=500);
    bool GenConfigMessage(const rapidjson::Value& Sensor, uint8_t node_address);
    void ConfigureSensors(const rapidjson::Value& Config, uint8_t node_address);
    bool ConfigureMissionManager(const rapidjson::Value& Config);
    bool ConfigureControlLaws(const rapidjson::Value& Config);
    void ConfigureEffectors(const rapidjson::Value& Config, uint8_t node_address);
    void RegisterSensors(const rapidjson::Value& Config);
    std::string GetSensorOutputName(const rapidjson::Value& Config,std::string Key,size_t index);
    void SendMessage(Message message, uint8_t address, std::vector<uint8_t> &Payload);
    void SendMessage(uint8_t message, uint8_t address, uint8_t *Payload, int len);
    bool ReceiveMessage(uint8_t *message, std::vector<uint8_t> *Payload);
    void PublishSensors();
};

#endif
