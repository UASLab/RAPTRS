/*
fmu.cxx
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

#include "fmu.h"
#include "millis.h"
#include "math.h"
#include <string>
using std::to_string;
using std::cout;
using std::endl;

#include "fmu_messages.h"

/* Opens port to communicate with FMU. */
void FlightManagementUnit::Begin() {
  _serial = new HardwareSerial(Port_);
  _bus = new SerialLink(*_serial);
  _bus->begin(Baud_);
}

/* Updates FMU configuration given a JSON value and registers data with global defs */
void FlightManagementUnit::Configure(const rapidjson::Value& Config) {
  std::vector<uint8_t> Payload;

  // switch FMU to configuration mode
  SendModeCommand(kConfigMode);

  // clear the serial buffer
  _bus->checkReceived();
  while (_bus->available()>0) {
    _bus->read();
    _bus->checkReceived();
  }

  // configure FMU sensors
  if (Config.HasMember("Sensors")) {
    std::cout << "\t\tSending Sensors config to FMU..." << std::flush;
    ConfigureSensors(Config["Sensors"], 0);
    std::cout << "done!" << std::endl;
  }

  // configuring FMU mission manager
  if (Config.HasMember("Mission-Manager")) {
    std::cout << "\t\tSending Mission-Manager config to FMU..." << std::flush;
    ConfigureMissionManager(Config["Mission-Manager"]);
    std::cout << "done!" << std::endl;
  }

  // configuring FMU control laws
  if (Config.HasMember("Control")) {
    std::cout << "\t\tSending Control config to FMU..." << std::flush;
    ConfigureControlLaws(Config["Control"]);
    std::cout << "done!" << std::endl;
  }

  // configuring FMU effectors
  if (Config.HasMember("Effectors")) {
    std::cout << "\t\tSending Effector config to FMU..." << std::flush;
    ConfigureEffectors(Config["Effectors"], 0);
    std::cout << "done!" << std::endl;
  }

  // switch FMU to run mode
  SendModeCommand(kRunMode);
}

/* Sends a mode command to the FMU */
void FlightManagementUnit::SendModeCommand(Mode mode) {
  printf("sending mode change: %d\n", mode);
  message::command_mode_t msg;
  msg.mode = mode;
  msg.pack();
  SendMessage(msg.id, 0, msg.payload, msg.len);
}

/* Receive sensor data from FMU */
bool FlightManagementUnit::ReceiveSensorData(bool publish) {
  uint8_t message;
  std::vector<uint8_t> Payload;
  size_t PayloadLocation = 0;
  bool freshdata = 0;
  while (ReceiveMessage(&message,&Payload)) {
    // printf("received msg: %d\n", message);
    if ( message == message::data_compound_id ) {
      size_t counter = 0;
      size_t mpu9250_counter = 0;
      size_t ublox_counter = 0;
      size_t swift_counter = 0;
      size_t ams5915_counter = 0;
      size_t sbus_counter = 0;
      size_t analog_counter = 0;
      while ( counter <= Payload.size() - 3 ) {
        uint8_t id = Payload[counter++];
        uint8_t index = Payload[counter++];
        uint8_t len = Payload[counter++];
        if ( counter + len <= Payload.size() ) {
          if ( id == message::data_time_id ) {
            message::data_time_t msg;
            msg.unpack(Payload.data()+counter, len);
            SensorData_.Time_us = msg.time_us;
          } else if ( id == message::data_mpu9250_id ) {
            // note: full message assumed to be the internal mpu9250
            message::data_mpu9250_t msg;
            msg.unpack(Payload.data()+counter, len);
            SensorData_.InternalMpu9250.AccelX_mss = msg.AccelX_mss;
            SensorData_.InternalMpu9250.AccelY_mss = msg.AccelY_mss;
            SensorData_.InternalMpu9250.AccelZ_mss = msg.AccelZ_mss;
            SensorData_.InternalMpu9250.GyroX_rads = msg.GyroX_rads;
            SensorData_.InternalMpu9250.GyroY_rads = msg.GyroY_rads;
            SensorData_.InternalMpu9250.GyroZ_rads = msg.GyroZ_rads;
            SensorData_.InternalMpu9250.MagX_uT = msg.MagX_uT;
            SensorData_.InternalMpu9250.MagY_uT = msg.MagY_uT;
            SensorData_.InternalMpu9250.MagZ_uT = msg.MagZ_uT;
            SensorData_.InternalMpu9250.Temperature_C = msg.Temperature_C;
          } else if ( id == message::data_mpu9250_short_id ) {
            message::data_mpu9250_short_t msg;
            msg.unpack(Payload.data()+counter, len);
            size_t i = mpu9250_counter;
            SensorData_.Mpu9250[i].AccelX_mss = msg.AccelX_mss;
            SensorData_.Mpu9250[i].AccelY_mss = msg.AccelY_mss;
            SensorData_.Mpu9250[i].AccelZ_mss = msg.AccelZ_mss;
            SensorData_.Mpu9250[i].GyroX_rads = msg.GyroX_rads;
            SensorData_.Mpu9250[i].GyroY_rads = msg.GyroY_rads;
            SensorData_.Mpu9250[i].GyroZ_rads = msg.GyroZ_rads;
            mpu9250_counter++;
          } else if ( id == message::data_bme280_id ) {
            message::data_bme280_t msg;
            msg.unpack(Payload.data()+counter, len);
            SensorData_.InternalBme280.Pressure_Pa = msg.Pressure_Pa;
            SensorData_.InternalBme280.Temperature_C = msg.Temperature_C;
            SensorData_.InternalBme280.Humidity_RH = msg.Humidity_RH;
          } else if ( id == message::data_ublox_id ) {
            message::data_ublox_t msg;
            msg.unpack(Payload.data()+counter, len);
            size_t i = ublox_counter;
            SensorData_.uBlox[i].Fix = msg.Fix;
            SensorData_.uBlox[i].NumberSatellites = msg.NumberSatellites;
            SensorData_.uBlox[i].TOW = msg.TOW;
            SensorData_.uBlox[i].Year = msg.Year;
            SensorData_.uBlox[i].Month = msg.Month;
            SensorData_.uBlox[i].Day = msg.Day;
            SensorData_.uBlox[i].Hour = msg.Hour;
            SensorData_.uBlox[i].Min = msg.Min;
            SensorData_.uBlox[i].Sec = msg.Sec;
            SensorData_.uBlox[i].Latitude_rad = msg.Latitude_rad;
            SensorData_.uBlox[i].Longitude_rad = msg.Longitude_rad;
            SensorData_.uBlox[i].Altitude_m = msg.Altitude_m;
            SensorData_.uBlox[i].NorthVelocity_ms = msg.NorthVelocity_ms;
            SensorData_.uBlox[i].EastVelocity_ms = msg.EastVelocity_ms;
            SensorData_.uBlox[i].DownVelocity_ms = msg.DownVelocity_ms;
            SensorData_.uBlox[i].HorizontalAccuracy_m = msg.HorizontalAccuracy_m;
            SensorData_.uBlox[i].VerticalAccuracy_m = msg.VerticalAccuracy_m;
            SensorData_.uBlox[i].VelocityAccuracy_ms = msg.VelocityAccuracy_ms;
            SensorData_.uBlox[i].pDOP = msg.pDOP;
            ublox_counter++;
          } else if ( id == message::data_swift_id ) {
            size_t i = swift_counter;
            message::data_swift_t msg;
            msg.unpack(Payload.data()+counter, len);
            SensorData_.Swift[i].Static.status = msg.static_ReadStatus;
            SensorData_.Swift[i].Static.Pressure_Pa = msg.static_Pressure_Pa;
            SensorData_.Swift[i].Static.Temperature_C = msg.static_Temperature_C;
            SensorData_.Swift[i].Differential.status = msg.diff_ReadStatus;
            SensorData_.Swift[i].Differential.Pressure_Pa = msg.diff_Pressure_Pa;
            SensorData_.Swift[i].Differential.Temperature_C = msg.diff_Temperature_C;
            swift_counter++;
          } else if ( id == message::data_ams5915_id ) {
            int i = ams5915_counter;
            message::data_ams5915_t msg;
            msg.unpack(Payload.data()+counter, len);
            SensorData_.Ams5915[i].status = msg.ReadStatus;
            SensorData_.Ams5915[i].Pressure_Pa = msg.Pressure_Pa;
            SensorData_.Ams5915[i].Temperature_C = msg.Temperature_C;
            ams5915_counter++;
          } else if ( id == message::data_sbus_id ) {
            int i = sbus_counter;
            message::data_sbus_t msg;
            msg.unpack(Payload.data()+counter, len);
            for ( int j = 0; j < 16; j++ ) {
              SensorData_.Sbus[i].Channels[j] = msg.channels[j];
            }
            SensorData_.Sbus[i].FailSafe = msg.FailSafe;
            SensorData_.Sbus[i].LostFrames = msg.LostFrames;
            sbus_counter++;
          } else if ( id == message::data_analog_id ) {
            message::data_analog_t msg;
            msg.unpack(Payload.data()+counter, len);
            size_t i = analog_counter;
            SensorData_.Analog[i].CalibratedValue = msg.calibrated_value;
            analog_counter++;
          } else {
            printf("SensorNode received an unhandled message id: %d\n", id);
          }
        }
        counter += len;
      } // while processing compound message
      
      if ( publish ) {
          // copy the incoming sensor data into the definition tree
          PublishSensors();
      }

      return true;
      freshdata = 1;
    } // if compound message
  }
  return freshdata;

}

/* Sends effector commands to FMU */
void FlightManagementUnit::SendEffectorCommands(std::vector<float> Commands) {
  message::command_effectors_t msg;
  msg.num_active = Commands.size();
  for ( size_t i = 0; i < Commands.size(); i++ ) {
    msg.command[i] = Commands[i];
  }
  msg.pack();
  SendMessage(msg.id, 0, msg.payload, msg.len);
}

/* Wait for Ack message */
bool FlightManagementUnit::WaitForAck(uint8_t id, uint8_t subid, float timeout_millis) {
  uint8_t msg_id;
  std::vector<uint8_t> Payload;
  uint64_t start = millis();
  while ( millis() < start + timeout_millis ) {
    if ( ReceiveMessage(&msg_id, &Payload) ) {
      if ( msg_id == message::config_ack_id ) {
        message::config_ack_t msg;
	msg.unpack(Payload.data(), Payload.size());
	printf("  received ack: %d ", msg.ack_id);
	if ( id == msg.ack_id and subid == msg.ack_subid ) {
	  printf("ok\n");
	  return true;
	} else {
	  printf("wrong ack\n");
	}
      }
    }
  }
  printf("Timeout waiting for ack: %d (%d)\n", id, subid);
  return false;
}

/* Generate sensor config messages */
bool FlightManagementUnit::GenConfigMessage(const rapidjson::Value& Sensor, uint8_t node_address) {
  if ( Sensor["Type"] == "Time" ) {
    if ( node_address == 0 ) {
      printf("Configuring Time\n");
      {
        string Path = RootPath_ + "/" + Sensor["Output"].GetString();
        SensorNodes_.Time_us = deftree.initElement(Path, "Flight management unit time, us", LOG_UINT64, LOG_NONE);
      }
      message::config_basic_t msg;
      msg.sensor = message::sensor_type::time;
      msg.output = Sensor["Output"].GetString();
      msg.pack();
      SendMessage(msg.id, 0, msg.payload, msg.len);
      if ( WaitForAck(msg.id, 0, 1000) ) {
        return true;
      }
    } else {
      printf("Cannot configure Time on a Node\n");
    }
  } else if ( Sensor["Type"] == "InputVoltage" ) {
    if ( node_address == 0 ) {
      printf("Configuring InputVoltage\n");
      {
        string Path = RootPath_+ "/" + Sensor["Output"].GetString();
        SensorNodes_.Analog.push_back(AnalogSensorNodes());
        SensorNodes_.Analog.back().val = deftree.initElement(Path, "InputVoltage_V", LOG_FLOAT, LOG_NONE);
        SensorData_.Analog.push_back(AnalogSensorData());
      }
      message::config_basic_t msg;
      msg.sensor = message::sensor_type::input_voltage;
      msg.output = Sensor["Output"].GetString();
      msg.pack();
      SendMessage(msg.id, 0, msg.payload, msg.len);
      if ( WaitForAck(msg.id, 0, 1000) ) {
        return true;
      }
    } else {
      printf("Cannot configure InputVoltage on a Node\n");
    }
  } else if ( Sensor["Type"] == "RegulatedVoltage" ) {
    if ( node_address == 0 ) {
      printf("Configuring RegulatedVoltage\n");
      {
        string Path = RootPath_+ "/" + Sensor["Output"].GetString();
        SensorNodes_.Analog.push_back(AnalogSensorNodes());
        SensorNodes_.Analog.back().val = deftree.initElement(Path, "RegulatedVoltage_V", LOG_FLOAT, LOG_NONE);
        SensorData_.Analog.push_back(AnalogSensorData());
      }
      message::config_basic_t msg;
      msg.sensor = message::sensor_type::regulated_voltage;
      msg.output = Sensor["Output"].GetString();
      msg.pack();
      SendMessage(msg.id, 0, msg.payload, msg.len);
      if ( WaitForAck(msg.id, 0, 1000) ) {
        return true;
      }
    } else {
      printf("Cannot configure RegulatedVoltage on a Node\n");
    }
  } else if ( Sensor["Type"] == "PwmVoltage" ) {
    printf("Configuring PwmVoltage\n");
    {
      string Path = RootPath_+ "/" + Sensor["Output"].GetString();
      SensorNodes_.Analog.push_back(AnalogSensorNodes());
      SensorNodes_.Analog.back().val = deftree.initElement(Path, "PwmVoltage_V", LOG_FLOAT, LOG_NONE);
      SensorData_.Analog.push_back(AnalogSensorData());
    }
    message::config_basic_t msg;
    msg.sensor = message::sensor_type::pwm_voltage;
    msg.output = Sensor["Output"].GetString();
    msg.pack();
    SendMessage(msg.id, node_address, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else if ( Sensor["Type"] == "SbusVoltage" ) {
    printf("Configuring SbusVoltage\n");
    {
      string Path = RootPath_+ "/" + Sensor["Output"].GetString();
      SensorNodes_.Analog.push_back(AnalogSensorNodes());
      SensorNodes_.Analog.back().val = deftree.initElement(Path, "SbusVoltage_V", LOG_FLOAT, LOG_NONE);
      SensorData_.Analog.push_back(AnalogSensorData());
    }
    message::config_basic_t msg;
    msg.sensor = message::sensor_type::sbus_voltage;
    msg.output = Sensor["Output"].GetString();
    msg.pack();
    SendMessage(msg.id, node_address, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else if ( Sensor["Type"] == "InternalBme280" ) {
    if ( node_address == 0 ) {
      printf("Configuring InternalBme280\n");
      {
        string Path = RootPath_ + "/" + Sensor["Output"].GetString();
        SensorNodes_.InternalBme280.press = deftree.initElement(Path+"/Pressure_Pa", "Flight management unit BME-280 static pressure, Pa", LOG_FLOAT, LOG_NONE);
        SensorNodes_.InternalBme280.temp = deftree.initElement(Path+"/Temperature_C", "Flight management unit BME-280 temperature, C", LOG_FLOAT, LOG_NONE);
        SensorNodes_.InternalBme280.hum = deftree.initElement(Path+"/Humidity_RH", "Flight management unit BME-280 percent relative humidity", LOG_FLOAT, LOG_NONE);
      }
      message::config_basic_t msg;
      msg.sensor = message::sensor_type::internal_bme280;
      msg.output = Sensor["Output"].GetString();
      msg.pack();
      SendMessage(msg.id, 0, msg.payload, msg.len);
      if ( WaitForAck(msg.id, 0, 1000) ) {
        return true;
      }
    } else {
      printf("Cannot configure InternalBme280 on a Node\n");
    }
  } else if ( Sensor["Type"] == "Sbus" ) {
    printf("Configuring Sbus\n");
    {
      string Path = RootPath_ + "/" + Sensor["Output"].GetString();
      SensorNodes_.Sbus.push_back(SbusSensorNodes());
      size_t i = SensorNodes_.Sbus.size() - 1;
      SensorNodes_.Sbus[i].failsafe = deftree.initElement(Path+"/FailSafe", "SBUS_" + to_string(i) + " fail safe status", LOG_UINT8, LOG_NONE);
      SensorNodes_.Sbus[i].lost_frames = deftree.initElement(Path+"/LostFrames", "SBUS_" + to_string(i) + " number of lost frames", LOG_UINT64, LOG_NONE);
      for (size_t j=0; j < 16; j++) {
        SensorNodes_.Sbus[i].ch[j] = deftree.initElement(Path+"/Channels/"+to_string(j), "SBUS_" + to_string(i) + " channel" + to_string(j) + " normalized value", LOG_FLOAT, LOG_NONE);
      }
      SensorData_.Sbus.push_back(SbusSensorData());
    }
    message::config_basic_t msg;
    msg.sensor = message::sensor_type::sbus;
    msg.output = Sensor["Output"].GetString();
    msg.pack();
    SendMessage(msg.id, node_address, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else if ( Sensor["Type"] == "InternalMpu9250" or Sensor["Type"] == "Mpu9250" ) {
    message::config_mpu9250_t msg;
    msg.output = Sensor["Output"].GetString();
    msg.orientation[0] = 1.0;
    msg.orientation[1] = 0.0;
    msg.orientation[2] = 0.0;
    msg.orientation[3] = 0.0;
    msg.orientation[4] = 1.0;
    msg.orientation[5] = 0.0;
    msg.orientation[6] = 0.0;
    msg.orientation[7] = 0.0;
    msg.orientation[8] = 1.0;
    if ( Sensor.HasMember("Rotation") ) {
      if ( Sensor["Rotation"].IsArray() and Sensor["Rotation"].Size() == 9 ) {
        for ( int i = 0; i < 9; i++ ) {
          msg.orientation[i] = Sensor["Rotation"][i].GetFloat();
        }
      } else {
        printf("ERROR: InternalMpu9250 Rotation incorrect\n");
      }
    }
    msg.DLPF_bandwidth_hz = 0;
    if ( Sensor.HasMember("DLPF-Bandwidth") ) {
      string bandwidth = Sensor["DLPF-Bandwidth"].GetString();
      if ( bandwidth == "184Hz" ) msg.DLPF_bandwidth_hz = 184;
      else if ( bandwidth == "92Hz" ) msg.DLPF_bandwidth_hz = 92;
      else if ( bandwidth == "41Hz" ) msg.DLPF_bandwidth_hz = 41;
      else if ( bandwidth == "20Hz" ) msg.DLPF_bandwidth_hz = 20;
      else if ( bandwidth == "10Hz" ) msg.DLPF_bandwidth_hz = 10;
      else if ( bandwidth == "5Hz" ) msg.DLPF_bandwidth_hz = 5;
      else {
        printf("ERROR: InternalMpu9250 DLPF-Bandwidth set incorrectly\n");
      }
    }
    if ( Sensor["Type"] == "InternalMpu9250" ) {
      if ( node_address == 0 ) {
        printf("Configuring InternalMpu9250\n");
        {
          string Path = RootPath_ + "/" + Sensor["Output"].GetString();
          SensorNodes_.InternalMpu9250.ax = deftree.initElement(Path+"/AccelX_mss", "Flight management unit MPU-9250 X accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.ay = deftree.initElement(Path+"/AccelY_mss", "Flight management unit MPU-9250 Y accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.az = deftree.initElement(Path+"/AccelZ_mss", "Flight management unit MPU-9250 Z accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.p = deftree.initElement(Path+"/GyroX_rads", "Flight management unit MPU-9250 X gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.q = deftree.initElement(Path+"/GyroY_rads", "Flight management unit MPU-9250 Y gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.r = deftree.initElement(Path+"/GyroZ_rads", "Flight management unit MPU-9250 Z gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.hx = deftree.initElement(Path+"/MagX_uT", "Flight management unit MPU-9250 X magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.hy = deftree.initElement(Path+"/MagY_uT", "Flight management unit MPU-9250 Y magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.hz = deftree.initElement(Path+"/MagZ_uT", "Flight management unit MPU-9250 Z magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
          SensorNodes_.InternalMpu9250.temp = deftree.initElement(Path+"/Temperature_C", "Flight management unit MPU-9250 temperature, C", LOG_FLOAT, LOG_NONE);
        }
        msg.internal = true;
      } else {
        printf("ERROR: Cannot configure InternalMpu9250 on a Node\n");
        return false;
      }
      if ( Sensor.HasMember("SRD") ) {
        msg.SRD = Sensor["SRD"].GetInt();
      } else {
        msg.SRD = 0;
      }
    } else {
      {
        SensorNodes_.Sbus.push_back(SbusSensorNodes());
        size_t i = SensorNodes_.Sbus.size() - 1;
        string Path = RootPath_ + "/" + Sensor["Output"].GetString();
        SensorNodes_.Mpu9250[i].status = deftree.initElement(Path+"/Status", "MPU-9250_" + to_string(i) + " read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
        SensorNodes_.Mpu9250[i].ax = deftree.initElement(Path+"/AccelX_mss", "MPU-9250_" + to_string(i) + " X accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
        SensorNodes_.Mpu9250[i].ay = deftree.initElement(Path+"/AccelY_mss", "MPU-9250_" + to_string(i) + " Y accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
        SensorNodes_.Mpu9250[i].az = deftree.initElement(Path+"/AccelZ_mss", "MPU-9250_" + to_string(i) + " Z accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
        SensorNodes_.Mpu9250[i].p = deftree.initElement(Path+"/GyroX_rads", "MPU-9250_" + to_string(i) + " X gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
        SensorNodes_.Mpu9250[i].q = deftree.initElement(Path+"/GyroY_rads", "MPU-9250_" + to_string(i) + " Y gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
        SensorNodes_.Mpu9250[i].r = deftree.initElement(Path+"/GyroZ_rads", "MPU-9250_" + to_string(i) + " Z gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
        // SensorNodes_.Mpu9250[i].hx = deftree.initElement(Path+"/MagX_uT", "MPU-9250_" + to_string(i) + " X magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
        // SensorNodes_.Mpu9250[i].hy = deftree.initElement(Path+"/MagY_uT", "MPU-9250_" + to_string(i) + " Y magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
        // SensorNodes_.Mpu9250[i].hz = deftree.initElement(Path+"/MagZ_uT", "MPU-9250_" + to_string(i) + " Z magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
        // SensorNodes_.Mpu9250[i].temp = deftree.initElement(Path+"/Temperature_C", "MPU-9250_" + to_string(i) + " temperature, C", LOG_FLOAT, LOG_NONE);
        SensorData_.Mpu9250.push_back(Mpu9250SensorData());
      }
      msg.internal = false;
      if ( Sensor.HasMember("UseSpi") ) {
        msg.use_spi = Sensor["UseSpi"].GetInt();
      }
      if ( Sensor.HasMember("Spi") ) {
        msg.spi_bus = Sensor["Spi"].GetInt();
      }
      if ( Sensor.HasMember("CsPin") ) {
        msg.cs_pin = Sensor["CsPin"].GetInt();
      } else if ( msg.use_spi ) {
        printf("ERROR: no spi pin specified\n");
      }
      if ( Sensor.HasMember("MosiPin" ) ) {
        msg.mosi_pin = Sensor["MosiPin"].GetInt();
      }
      if ( Sensor.HasMember("MisoPin" ) ) {
        msg.miso_pin = Sensor["MisoPin"].GetInt();
      }
      if ( Sensor.HasMember("SckPin" ) ) {
        msg.sck_pin = Sensor["SckPin"].GetInt();
      }
      if ( Sensor.HasMember("Address") ) {
        msg.i2c_addr = Sensor["Address"].GetInt();
      } else if ( !msg.use_spi ) {
        printf("ERROR: no i2c address specified\n");
      }
      if ( Sensor.HasMember("I2c") ) {
        msg.i2c_bus = Sensor["I2c"].GetInt();
      }
    }
    msg.pack();
    SendMessage(msg.id, 0, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else if ( Sensor["Type"] == "uBlox" ) {
    printf("Configuring uBlox\n");
    {
      string Path = RootPath_ + "/" + Sensor["Output"].GetString();
      SensorNodes_.uBlox.push_back(uBloxSensorNodes());
      size_t i = SensorNodes_.uBlox.size() - 1;
      SensorNodes_.uBlox[i].fix = deftree.initElement(Path+"/Fix", "uBlox_" + to_string(i) + " fix status, true for 3D fix only", LOG_UINT8, LOG_NONE);
      SensorNodes_.uBlox[i].sats = deftree.initElement(Path+"/NumberSatellites", "uBlox_" + to_string(i) + " number of satellites used in solution", LOG_UINT8, LOG_NONE);
      SensorNodes_.uBlox[i].tow = deftree.initElement(Path+"/TOW", "uBlox_" + to_string(i) + " GPS time of the navigation epoch", LOG_UINT32, LOG_NONE);
      SensorNodes_.uBlox[i].year = deftree.initElement(Path+"/Year", "uBlox_" + to_string(i) + " UTC year", LOG_UINT16, LOG_NONE);
      SensorNodes_.uBlox[i].month = deftree.initElement(Path+"/Month", "uBlox_" + to_string(i) + " UTC month", LOG_UINT8, LOG_NONE);
      SensorNodes_.uBlox[i].day = deftree.initElement(Path+"/Day", "uBlox_" + to_string(i) + " UTC day", LOG_UINT8, LOG_NONE);
      SensorNodes_.uBlox[i].hour = deftree.initElement(Path+"/Hour", "uBlox_" + to_string(i) + " UTC hour", LOG_UINT8, LOG_NONE);
      SensorNodes_.uBlox[i].min = deftree.initElement(Path+"/Minute", "uBlox_" + to_string(i) + " UTC minute", LOG_UINT8, LOG_NONE);
      SensorNodes_.uBlox[i].sec = deftree.initElement(Path+"/Second", "uBlox_" + to_string(i) + " UTC second", LOG_UINT8, LOG_NONE);
      SensorNodes_.uBlox[i].lat = deftree.initElement(Path+"/Latitude_rad", "uBlox_" + to_string(i) + " latitude, rad", LOG_DOUBLE, LOG_NONE);
      SensorNodes_.uBlox[i].lon = deftree.initElement(Path+"/Longitude_rad", "uBlox_" + to_string(i) + " longitude, rad", LOG_DOUBLE, LOG_NONE);
      SensorNodes_.uBlox[i].alt = deftree.initElement(Path+"/Altitude_m", "uBlox_" + to_string(i) + " altitude above mean sea level, m", LOG_FLOAT, LOG_NONE);
      SensorNodes_.uBlox[i].vn = deftree.initElement(Path+"/NorthVelocity_ms", "uBlox_" + to_string(i) + " north velocity, m/s", LOG_FLOAT, LOG_NONE);
      SensorNodes_.uBlox[i].ve = deftree.initElement(Path+"/EastVelocity_ms", "uBlox_" + to_string(i) + " east velocity, m/s", LOG_FLOAT, LOG_NONE);
      SensorNodes_.uBlox[i].vd = deftree.initElement(Path+"/DownVelocity_ms", "uBlox_" + to_string(i) + " down velocity, m/s", LOG_FLOAT, LOG_NONE);
      SensorNodes_.uBlox[i].horiz_acc = deftree.initElement(Path+"/HorizontalAccuracy_m", "uBlox_" + to_string(i) + " horizontal accuracy estimate, m", LOG_FLOAT, LOG_NONE);
      SensorNodes_.uBlox[i].vert_acc = deftree.initElement(Path+"/VerticalAccuracy_m", "uBlox_" + to_string(i) + " vertical accuracy estimate, m", LOG_FLOAT, LOG_NONE);
      SensorNodes_.uBlox[i].vel_acc = deftree.initElement(Path+"/VelocityAccuracy_ms", "uBlox_" + to_string(i) + " velocity accuracy estimate, m/s", LOG_FLOAT, LOG_NONE);
      SensorNodes_.uBlox[i].pdop = deftree.initElement(Path+"/pDOP", "uBlox_" + to_string(i) + " position dilution of precision", LOG_FLOAT, LOG_NONE);
      SensorData_.uBlox.push_back(uBloxSensorData());
    }
    message::config_ublox_t msg;
    msg.output = Sensor["Output"].GetString();
    if ( Sensor.HasMember("Uart") ) {
      msg.uart = Sensor["Uart"].GetInt();
    } else {
      msg.uart = 0;
    }
    if ( Sensor.HasMember("Baud") ) {
      msg.baud = Sensor["Baud"].GetInt();
    } else {
      msg.baud = 0;
    }
    printf("uBlox: %d %d\n", msg.uart, msg.baud);
    msg.pack();
    SendMessage(msg.id, node_address, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else if ( Sensor["Type"] == "Swift" ) {
    printf("Configuring Swift\n");
    {
      string Path = RootPath_ + "/" + Sensor["Output"].GetString();
      SensorNodes_.Swift.push_back(SwiftSensorNodes());
      size_t i = SensorNodes_.Swift.size() - 1;
      SensorNodes_.Swift[i].Static.status = deftree.initElement(Path+"/Static/Status", "Swift_" + to_string(i) + " static pressure read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
      SensorNodes_.Swift[i].Static.press = deftree.initElement(Path+"/Static/Pressure_Pa", "Swift_" + to_string(i) + " static pressure, Pa", LOG_FLOAT, LOG_NONE);
      SensorNodes_.Swift[i].Static.temp = deftree.initElement(Path+"/Static/Temperature_C", "Swift_" + to_string(i) + " static pressure transducer temperature, C", LOG_FLOAT, LOG_NONE);
      SensorNodes_.Swift[i].Differential.status = deftree.initElement(Path+"/Differential/Status", "Swift_" + to_string(i) + " differential pressure read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
      SensorNodes_.Swift[i].Differential.press = deftree.initElement(Path+"/Differential/Pressure_Pa", "Swift_" + to_string(i) + " differential pressure, Pa", LOG_FLOAT, LOG_NONE);
      SensorNodes_.Swift[i].Differential.temp = deftree.initElement(Path+"/Differential/Temperature_C", "Swift_" + to_string(i) + " differential pressure transducer temperature, C", LOG_FLOAT, LOG_NONE);
      SensorData_.Swift.push_back(SwiftSensorData());
    }
    message::config_swift_t msg;
    msg.output = Sensor["Output"].GetString();
    if ( Sensor.HasMember("I2c") ) {
      msg.i2c_bus = Sensor["I2c"].GetInt();
    } else {
      msg.i2c_bus = 1;
    }
    if ( Sensor.HasMember("Static") ) {
      const rapidjson::Value &st = Sensor["Static"];
      if ( st.HasMember("Address") ) {
        msg.static_i2c_addr = st["Address"].GetInt();
      } else {
        printf("ERROR: Swift Static config missing Address\n");
      }
    } else {
      printf("ERROR: Swift config missing Static section\n");
    }
    if ( Sensor.HasMember("Differential") ) {
      const rapidjson::Value &diff = Sensor["Differential"];
      if ( diff.HasMember("Address") ) {
        msg.diff_i2c_addr = diff["Address"].GetInt();
      } else {
        printf("ERROR: Swift Differential config missing Address\n");
      }
      if ( diff.HasMember("Transducer") ) {
        msg.diff_transducer = diff["Transducer"].GetString();
      } else {
        printf("ERROR: Swift Differential config missing Transducer\n");
      }
    } else {
      printf("ERROR: Swift config missing Differential section\n");
    }
    msg.pack();
    SendMessage(msg.id, node_address, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else if ( Sensor["Type"] == "Bme280" ) {
    printf("Configuring Bme280\n");
    {
      string Path = RootPath_ + "/" + Sensor["Output"].GetString();
      SensorNodes_.Bme280.push_back(Bme280SensorNodes());
      size_t i = SensorNodes_.Bme280.size() - 1;
      SensorNodes_.Bme280[i].status = deftree.initElement(Path+"/Status", "BME-280_" + to_string(i) + " read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
      SensorNodes_.Bme280[i].press = deftree.initElement(Path+"/Pressure_Pa", "BME-280_" + to_string(i) + " static pressure, Pa", LOG_FLOAT, LOG_NONE);
      SensorNodes_.Bme280[i].temp = deftree.initElement(Path+"/Temperature_C", "BME-280_" + to_string(i) + " temperature, C", LOG_FLOAT, LOG_NONE);
      SensorNodes_.Bme280[i].hum = deftree.initElement(Path+"/Humidity_RH", "BME-280_" + to_string(i) + " percent relative humidity", LOG_FLOAT, LOG_NONE);
      SensorData_.Bme280.push_back(Bme280SensorData());
    }
    message::config_bme280_t msg;
    msg.output = Sensor["Output"].GetString();
    if ( Sensor.HasMember("UseSpi") ) {
      msg.use_spi = Sensor["UseSpi"].GetInt();
    }
    if ( Sensor.HasMember("CsPin") ) {
      msg.cs_pin = Sensor["CsPin"].GetInt();
    } else if ( msg.use_spi ) {
      printf("ERROR: no spi pin specified\n");
    }
    if ( Sensor.HasMember("Address") ) {
      msg.i2c_addr = Sensor["Address"].GetInt();
    } else if ( !msg.use_spi ) {
      printf("ERROR: no i2c address specified\n");
    }      
    msg.pack();
    SendMessage(msg.id, 0, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else if ( Sensor["Type"] == "Ams5915" ) {
    printf("Configuring Ams5915\n");
    {
      string Path = RootPath_ + "/" + Sensor["Output"].GetString();
      SensorNodes_.Ams5915.push_back(Ams5915SensorNodes());
      size_t i = SensorNodes_.Ams5915.size() - 1;
      SensorNodes_.Ams5915[i].status = deftree.initElement(Path+"/Status", "AMS-5915_" + to_string(i) + " read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
      SensorNodes_.Ams5915[i].press = deftree.initElement(Path+"/Pressure_Pa", "AMS-5915_" + to_string(i) + " pressure, Pa", LOG_FLOAT, LOG_NONE);
      SensorNodes_.Ams5915[i].temp = deftree.initElement(Path+"/Temperature_C", "AMS-5915_" + to_string(i) + " pressure transducer temperature, C", LOG_FLOAT, LOG_NONE);
      SensorData_.Ams5915.push_back(Ams5915SensorData());
    }
    message::config_ams5915_t msg;
    msg.output = Sensor["Output"].GetString();
    if ( Sensor.HasMember("I2c") ) {
      msg.i2c_bus = Sensor["I2c"].GetInt();
    }
    if ( Sensor.HasMember("Address") ) {
      msg.i2c_addr = Sensor["Address"].GetInt();
    } else {
        printf("ERROR: no i2c address specified");
    }
    if ( Sensor.HasMember("Transducer") ) {
      msg.transducer = Sensor["Transducer"].GetString();
    } else {
      printf("ERROR: no transducer type specified\n");
    }
    msg.pack();
    SendMessage(msg.id, 0, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else if ( Sensor["Type"] == "Analog" ) {
    printf("Configuring Analog\n");
    {
      string Path = RootPath_ + "/" + Sensor["Output"].GetString();
      SensorNodes_.Analog.push_back(AnalogSensorNodes());
      size_t i = SensorNodes_.Analog.size() - 1;
      SensorNodes_.Analog[i].val = deftree.initElement(Path+"/CalibratedValue", "Analog_" + to_string(i) + " calibrated value", LOG_FLOAT, LOG_NONE);
      SensorData_.Analog.push_back(AnalogSensorData());
    }
    message::config_analog_t msg;
    msg.output = Sensor["Output"].GetString();
    if ( Sensor.HasMember("Channel") ) {
      msg.channel = Sensor["Channel"].GetInt();
    } else {
        printf("ERROR: no analog channel specified");
    }
    msg.calibration[0] = nanf("");
    msg.calibration[1] = nanf("");
    msg.calibration[2] = nanf("");
    msg.calibration[3] = nanf("");
    if ( Sensor.HasMember("Calibration") ) {
      if ( Sensor["Calibration"].IsArray() and Sensor["Calibration"].Size() <= 4 ) {
        for ( size_t i = 0; i < Sensor["Calibration"].Size(); i++ ) {
          msg.calibration[i] = Sensor["Calibration"][i].GetFloat();
        }
      } else {
        printf("ERROR: analog calibration incorrect\n");
      }
    }
    msg.pack();
    SendMessage(msg.id, 0, msg.payload, msg.len);
    if ( WaitForAck(msg.id, 0, 1000) ) {
      return true;
    }
  } else {
    printf("ERROR: unknown sensor\n");
  }
  return false;
}

/* Configures the FMU sensors */
void FlightManagementUnit::ConfigureSensors(const rapidjson::Value& Config, uint8_t node_address) {
  std::vector<uint8_t> Payload;
  assert(Config.IsArray());
  for (size_t i=0; i < Config.Size(); i++) {
    const rapidjson::Value& Sensor = Config[i];
    if ( Sensor.HasMember("Type") ) {
      if ( Sensor["Type"] == "Node" ) {
        if ( Sensor.HasMember("Address") and Sensor.HasMember("Sensors") ) {
          ConfigureSensors(Sensor["Sensors"], Sensor["Address"].GetInt());
        } else {
          printf("ERROR: sensor node specified without a valid address or sensor sub block\n");
        }
      } else if ( GenConfigMessage(Sensor, node_address) ) {
	// new way succeeded!
      } else {
	printf("ERROR: failed to generate config message: %s\n", Sensor["Type"].GetString() );
      }
    }
  }
}

/* Configures the FMU mission manager */
bool FlightManagementUnit::ConfigureMissionManager(const rapidjson::Value& Config) {
  message::config_mission_t msg;
  if ( Config.HasMember("Fmu-Soc-Switch") ) {
    const rapidjson::Value &sw = Config["Fmu-Soc-Switch"];
    msg.switch_name = "Fmu-Soc-Switch";
    if ( sw.HasMember("Source") ) {
      msg.source = sw["Source"].GetString();
    } else {
      printf("ERROR: no fmu-soc-switch source field specified\n");
    }
    if ( sw.HasMember("Gain") ) {
      msg.gain = sw["Gain"].GetFloat();
    }
    if ( sw.HasMember("Threshold") ) {
      msg.threshold = sw["Threshold"].GetFloat();
    }
    msg.pack();
    SendMessage(msg.id, 0, msg.payload, msg.len);
    if ( ! WaitForAck(msg.id, 0, 1000) ) {
      return false;
    }
  }
  if ( Config.HasMember("Throttle-Safety-Switch") ) {
    const rapidjson::Value &sw = Config["Throttle-Safety-Switch"];
    msg.switch_name = "Throttle-Safety-Switch";
    if ( sw.HasMember("Source") ) {
      msg.source = sw["Source"].GetString();
    } else {
      printf("ERROR: no throttle-safety-switch source field specified\n");
    }
    if ( sw.HasMember("Gain") ) {
      msg.gain = sw["Gain"].GetFloat();
    }
    if ( sw.HasMember("Threshold") ) {
      msg.threshold = sw["Threshold"].GetFloat();
    }
    msg.pack();
    SendMessage(msg.id, 0, msg.payload, msg.len);
    if ( ! WaitForAck(msg.id, 0, 1000) ) {
      return false;
    }
  }
  return true;
}

/* Configures the FMU control laws */
/* example JSON configuration:
{
  "Output": "OutputName",
  "Input": "InputName",
  "Gain": X,
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where OutputName gives a convenient name for the block (i.e. SpeedControl).
Input is the full path name of the input signal
Gain is the gain applied to the input signal
Limits are optional and saturate the output if defined
*/
bool FlightManagementUnit::ConfigureControlLaws(const rapidjson::Value& Config) {
  std::vector<uint8_t> Payload;
  if ( Config.HasMember("Fmu") ) {
    std::string block_name = Config["Fmu"].GetString();
    if ( Config.HasMember(block_name.c_str()) ) {
      if ( Config[block_name.c_str()].IsArray() ) {
        // iterate through levels
        for ( size_t j = 0; j < Config[block_name.c_str()].Size(); j++ ) {
          const rapidjson::Value &Level = Config[block_name.c_str()][j];
          if ( Level.HasMember("Level") and Level.HasMember("Components") and Level["Components"].IsArray() ) {
            printf("Control laws size: %d\n", Level["Components"].Size());
            for ( size_t i = 0; i < Level["Components"].Size(); i++ ) {
              printf("  component: %d\n", i);
              const rapidjson::Value &Component = Level["Components"][i];
              if ( Component.HasMember("Type") and Component["Type"] == "Gain" ) {
                message::config_control_gain_t msg;
                if ( Component.HasMember("Input") ) {
                  msg.input = Component["Input"].GetString();
                } else {
                  printf("ERROR: component does not specified an input\n");
                }
                if ( Component.HasMember("Output") ) {
                  msg.output = Component["Output"].GetString();
                } else {
                  printf("ERROR: component does not specified an output\n");
                }
                if ( Component.HasMember("Gain") ) {
                  msg.gain = Component["Gain"].GetFloat();
                } else {
                  printf("ERROR: component does not specified a gain\n");
                }
                if ( Component.HasMember("Limits") ) {
                  msg.has_limits = true;
                  const rapidjson::Value &Limits = Component["Limits"];
                  if ( Limits.HasMember("Lower") ) {
                    msg.lower_limit = Limits["Lower"].GetFloat();
                  } else {
                    printf("ERROR: no lower limit in control\n");
                  }
                  if ( Limits.HasMember("Upper") ) {
                    msg.upper_limit = Limits["Upper"].GetFloat();
                  } else {
                    printf("ERROR: no upper limit in control\n");
                  }
                } else {
                  msg.has_limits = false;
                }
                msg.pack();
                SendMessage(msg.id, 0, msg.payload, msg.len);
                if ( !WaitForAck(msg.id, 0, 1000) ) {
                  return false;
                }
              } else {
                printf("ignore component that isn't a gain\n");
              }
            }
            // success if we made it to here
            return true;
          } else {
            printf("ERROR: Level or Components not specified correctly in Control\n");
          }
        }
      } else {
        printf("ERROR: %s not an array\n", block_name.c_str());
      }
    } else {
      printf("ERROR: could not find baseline control law: %s\n", block_name.c_str());
    }
  } else {
    printf("ERROR: Fmu key not found in Control\n");
  }
  return false;
}

/* Configures the FMU effectors */
void FlightManagementUnit::ConfigureEffectors(const rapidjson::Value& Config, uint8_t node_address) {
  std::vector<uint8_t> Payload;
  assert(Config.IsArray());
  for (size_t i=0; i < Config.Size(); i++) {
    const rapidjson::Value& Effector = Config[i];
    if ( Effector.HasMember("Type" ) ) {
      if ( Effector["Type"] == "Node" ) {
        if ( Effector.HasMember("Address") and Effector.HasMember("Effectors") ) {
          ConfigureEffectors(Effector["Effectors"], Effector["Address"].GetInt());
        } else {
          printf("ERROR: effector node specified without a valid address or effectors sub block\n");
        }
      } else {
        message::config_effector_t msg;
        if ( Effector["Type"] == "Motor" ) {
          msg.effector = message::effector_type::motor;
        } else if ( Effector["Type"] == "Pwm" ) {
          msg.effector = message::effector_type::pwm;
        } else if ( Effector["Type"] == "Sbus" ) {
          msg.effector = message::effector_type::sbus;
        } else {
          printf("ERROR: effector without a valid type\n");
        }
        if ( Effector.HasMember("Input") ) {
          msg.input = Effector["Input"].GetString();
        } else {
          printf("ERROR: effector without Input defined\n");
        }
        if ( Effector.HasMember("Channel") ) {
          msg.channel = Effector["Channel"].GetInt();
        } else {
          printf("ERROR: effector without Channel defined\n");
        }
        msg.calibration[0] = nanf("");
        msg.calibration[1] = nanf("");
        msg.calibration[2] = nanf("");
        msg.calibration[3] = nanf("");
        if ( Effector.HasMember("Calibration") ) {
          if ( Effector["Calibration"].IsArray() and Effector["Calibration"].Size() <= message::max_calibration ) {
            for ( size_t i = 0; i < Effector["Calibration"].Size(); i++ ) {
              msg.calibration[i] = Effector["Calibration"][i].GetFloat();
            }
          } else {
            printf("ERROR: effector calibration incorrect\n");
          }
        }
        printf("eff calib: ");
        for ( int i = 0; i < message::max_calibration; i++ ) {
          printf("%f ", msg.calibration[i]);
        }
        printf("\n");
        if ( Effector.HasMember("Safed-Command") ) {
          msg.safed_command = Effector["Safed-Command"].GetInt();
        } else if ( Effector["Type"] == "Motor" ){
          printf("ERROR: effector type motor without a safed-command\n");
        }
        msg.pack();
        SendMessage(msg.id, node_address, msg.payload, msg.len);
        if ( ! WaitForAck(msg.id, 0, 1000) ) {
          printf("ERROR: effector command message failed ack!\n");
        }
      }
    } else {
      printf("ERROR: effector defined with no Type\n");
    }
  }
}

/* Send a (Serial) BFS Bus message. */
void FlightManagementUnit::SendMessage(Message message, uint8_t address, std::vector<uint8_t> &Payload) {
  SendMessage(message, address, Payload.data(), Payload.size());
}

/* Send a (Serial) BFS Bus message. */
void FlightManagementUnit::SendMessage(uint8_t message, uint8_t address, uint8_t *Payload, int len) {
  _bus->beginTransmission();
  _bus->write(message);
  _bus->write(address);
  _bus->write(Payload, len);
  if ((message == kModeCommand)||(message == kConfigMesg)) {
    _bus->endTransmission();
  } else {
    _bus->sendTransmission();
  }
}

/* Receive a BFS Bus message. */
bool FlightManagementUnit::ReceiveMessage(uint8_t *message, std::vector<uint8_t> *Payload) {
  if (_bus->checkReceived()) {
    *message = _bus->read();
    uint8_t address = _bus->read();
    // printf("received msg: %d (size = %d)\n", *message, _bus->available());
    Payload->resize(_bus->available());
    _bus->read(Payload->data(),Payload->size());
    _bus->sendStatus(true);
    return true;
  } else {
    return false;
  }
}

void FlightManagementUnit::PublishSensors() {
  SensorNodes_.Time_us->setLong(SensorData_.Time_us);
  SensorNodes_.InternalMpu9250.ax->setFloat(SensorData_.InternalMpu9250.AccelX_mss);
  SensorNodes_.InternalMpu9250.ay->setFloat(SensorData_.InternalMpu9250.AccelY_mss);
  SensorNodes_.InternalMpu9250.az->setFloat(SensorData_.InternalMpu9250.AccelZ_mss);
  SensorNodes_.InternalMpu9250.p->setFloat(SensorData_.InternalMpu9250.GyroX_rads);
  SensorNodes_.InternalMpu9250.q->setFloat(SensorData_.InternalMpu9250.GyroY_rads);
  SensorNodes_.InternalMpu9250.r->setFloat(SensorData_.InternalMpu9250.GyroZ_rads);
  SensorNodes_.InternalMpu9250.hx->setFloat(SensorData_.InternalMpu9250.MagX_uT);
  SensorNodes_.InternalMpu9250.hy->setFloat(SensorData_.InternalMpu9250.MagY_uT);
  SensorNodes_.InternalMpu9250.hz->setFloat(SensorData_.InternalMpu9250.MagZ_uT);
  SensorNodes_.InternalMpu9250.temp->setFloat(SensorData_.InternalMpu9250.Temperature_C);
  SensorNodes_.InternalBme280.press->setFloat(SensorData_.InternalBme280.Pressure_Pa);
  SensorNodes_.InternalBme280.temp->setFloat(SensorData_.InternalBme280.Temperature_C);
  SensorNodes_.InternalBme280.hum->setFloat(SensorData_.InternalBme280.Humidity_RH);
  for (size_t i=0; i < SensorData_.Mpu9250.size(); i++) {

    //const float G = 9.807f;
    //const float d2r = 3.14159265359f/180.0f;
    //float accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
    //float gyroScale = 2000.0f/32767.5f * d2r; // setting the gyro scale to 2000DPS

    SensorNodes_.Mpu9250[i].status->setInt(SensorData_.Mpu9250[i].status);
    SensorNodes_.Mpu9250[i].ax->setFloat((float) (SensorData_.Mpu9250[i].AccelX_mss));
    SensorNodes_.Mpu9250[i].ay->setFloat((float) (SensorData_.Mpu9250[i].AccelY_mss));
    SensorNodes_.Mpu9250[i].az->setFloat((float) (SensorData_.Mpu9250[i].AccelZ_mss));
    SensorNodes_.Mpu9250[i].p->setFloat((float) (SensorData_.Mpu9250[i].GyroX_rads));
    SensorNodes_.Mpu9250[i].q->setFloat((float) (SensorData_.Mpu9250[i].GyroY_rads));
    SensorNodes_.Mpu9250[i].r->setFloat((float) (SensorData_.Mpu9250[i].GyroZ_rads));
    // SensorNodes_.Mpu9250[i].hx->setFloat(SensorData_.Mpu9250[i].MagX_uT);
    // SensorNodes_.Mpu9250[i].hy->setFloat(SensorData_.Mpu9250[i].MagY_uT;
    // SensorNodes_.Mpu9250[i].hz->setFloat(SensorData_.Mpu9250[i].MagZ_uT);
    // SensorNodes_.Mpu9250[i].temp->setFloat(SensorData_.Mpu9250[i].Temperature_C);
  }
  for (size_t i=0; i < SensorData_.Bme280.size(); i++) {
    SensorNodes_.Bme280[i].status->setInt(SensorData_.Bme280[i].status);
    SensorNodes_.Bme280[i].press->setFloat(SensorData_.Bme280[i].Pressure_Pa);
    SensorNodes_.Bme280[i].temp->setFloat(SensorData_.Bme280[i].Temperature_C);
    SensorNodes_.Bme280[i].hum->setFloat(SensorData_.Bme280[i].Humidity_RH);
  }
  for (size_t i=0; i < SensorData_.uBlox.size(); i++) {
    SensorNodes_.uBlox[i].fix->setInt(SensorData_.uBlox[i].Fix);
    SensorNodes_.uBlox[i].sats->setInt(SensorData_.uBlox[i].NumberSatellites);
    SensorNodes_.uBlox[i].tow->setInt(SensorData_.uBlox[i].TOW);
    SensorNodes_.uBlox[i].year->setInt(SensorData_.uBlox[i].Year);
    SensorNodes_.uBlox[i].month->setInt(SensorData_.uBlox[i].Month);
    SensorNodes_.uBlox[i].day->setInt(SensorData_.uBlox[i].Day);
    SensorNodes_.uBlox[i].hour->setInt(SensorData_.uBlox[i].Hour);
    SensorNodes_.uBlox[i].min->setInt(SensorData_.uBlox[i].Min);
    SensorNodes_.uBlox[i].sec->setInt(SensorData_.uBlox[i].Sec);
    SensorNodes_.uBlox[i].lat->setDouble(SensorData_.uBlox[i].Latitude_rad);
    SensorNodes_.uBlox[i].lon->setDouble(SensorData_.uBlox[i].Longitude_rad);
    SensorNodes_.uBlox[i].alt->setDouble(SensorData_.uBlox[i].Altitude_m);
    SensorNodes_.uBlox[i].vn->setFloat(SensorData_.uBlox[i].NorthVelocity_ms);
    SensorNodes_.uBlox[i].ve->setFloat(SensorData_.uBlox[i].EastVelocity_ms);
    SensorNodes_.uBlox[i].vd->setFloat(SensorData_.uBlox[i].DownVelocity_ms);
    SensorNodes_.uBlox[i].horiz_acc->setFloat(SensorData_.uBlox[i].HorizontalAccuracy_m);
    SensorNodes_.uBlox[i].vert_acc->setFloat(SensorData_.uBlox[i].VerticalAccuracy_m);
    SensorNodes_.uBlox[i].vel_acc->setFloat(SensorData_.uBlox[i].VelocityAccuracy_ms);
    SensorNodes_.uBlox[i].pdop->setFloat(SensorData_.uBlox[i].pDOP);
  }
  for (size_t i=0; i < SensorData_.Swift.size(); i++) {
    SensorNodes_.Swift[i].Static.status->setInt(SensorData_.Swift[i].Static.status);
    SensorNodes_.Swift[i].Static.press->setFloat(SensorData_.Swift[i].Static.Pressure_Pa);
    SensorNodes_.Swift[i].Static.temp->setFloat(SensorData_.Swift[i].Static.Temperature_C);
    SensorNodes_.Swift[i].Differential.status->setInt(SensorData_.Swift[i].Differential.status);
    SensorNodes_.Swift[i].Differential.press->setFloat(SensorData_.Swift[i].Differential.Pressure_Pa);
    SensorNodes_.Swift[i].Differential.temp->setFloat(SensorData_.Swift[i].Differential.Temperature_C);
  }
  for (size_t i=0; i < SensorData_.Ams5915.size(); i++) {
    SensorNodes_.Ams5915[i].status->setInt(SensorData_.Ams5915[i].status);
    SensorNodes_.Ams5915[i].press->setFloat(SensorData_.Ams5915[i].Pressure_Pa);
    SensorNodes_.Ams5915[i].temp->setFloat(SensorData_.Ams5915[i].Temperature_C);
  }
  for (size_t i=0; i < SensorData_.Sbus.size(); i++) {
    SensorNodes_.Sbus[i].failsafe->setInt(SensorData_.Sbus[i].FailSafe);
    SensorNodes_.Sbus[i].lost_frames->setInt(SensorData_.Sbus[i].LostFrames);
    for (size_t j=0; j < 16; j++) {
      SensorNodes_.Sbus[i].ch[j]->setFloat(SensorData_.Sbus[i].Channels[j]);
    }
  }
  for (size_t i=0; i < SensorData_.Analog.size(); i++) {
    // SensorNodes_.Analog[i].volt->setFloat(SensorData_.Analog[i].Voltage_V);
    SensorNodes_.Analog[i].val->setFloat(SensorData_.Analog[i].CalibratedValue);
  }
}
