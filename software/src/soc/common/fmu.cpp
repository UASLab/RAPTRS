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

#include <string>
using std::to_string;
using std::cout;
using std::endl;

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

  // get the updated configuration from the sensor meta data
  std::cout << "\t\tReading Sensors config back from FMU..." << std::flush;
  size_t i=0;
  while(i < 100) {
    if (ReceiveSensorData(false /*publish*/)) {
      i++;
    }
  }
  std::cout << "done!" << std::endl;

  // register sensor data with global definition tree
  if (Config.HasMember("Sensors")) {
    std::cout << "\t\tRegistering Sensors with DefinitionTree..." << std::flush;
    RegisterSensors(Config["Sensors"]);
    std::cout << "done!" << std::endl;
  }
}

/* Sends a mode command to the FMU */
void FlightManagementUnit::SendModeCommand(Mode mode) {
  printf("sending mode change: %d\n", mode);
  cmd_msg.mode = mode;
  cmd_msg.pack();
  SendMessage(cmd_msg.id, 0, cmd_msg.payload, cmd_msg.len);
}

/* Receive sensor data from FMU */
bool FlightManagementUnit::ReceiveSensorData(bool publish) {
  uint8_t message;
  std::vector<uint8_t> Payload;
  size_t PayloadLocation = 0;
  bool freshdata = 0;
  while (ReceiveMessage(&message,&Payload)) {
    printf("received msg: %d\n", message);
    if (message == kSensorData) {
      // meta data
      uint8_t AcquireInternalData,NumberPwmVoltageSensor,NumberSbusVoltageSensor,NumberMpu9250Sensor,NumberBme280Sensor,NumberuBloxSensor,NumberSwiftSensor,NumberAms5915Sensor,NumberSbusSensor,NumberAnalogSensor;
      memcpy(&AcquireInternalData,Payload.data()+PayloadLocation,sizeof(AcquireInternalData));
      PayloadLocation += sizeof(AcquireInternalData);
      memcpy(&NumberPwmVoltageSensor,Payload.data()+PayloadLocation,sizeof(NumberPwmVoltageSensor));
      PayloadLocation += sizeof(NumberPwmVoltageSensor);
      memcpy(&NumberSbusVoltageSensor,Payload.data()+PayloadLocation,sizeof(NumberSbusVoltageSensor));
      PayloadLocation += sizeof(NumberSbusVoltageSensor);
      memcpy(&NumberMpu9250Sensor,Payload.data()+PayloadLocation,sizeof(NumberMpu9250Sensor));
      PayloadLocation += sizeof(NumberMpu9250Sensor);
      memcpy(&NumberBme280Sensor,Payload.data()+PayloadLocation,sizeof(NumberBme280Sensor));
      PayloadLocation += sizeof(NumberBme280Sensor);
      memcpy(&NumberuBloxSensor,Payload.data()+PayloadLocation,sizeof(NumberuBloxSensor));
      PayloadLocation += sizeof(NumberuBloxSensor);
      memcpy(&NumberSwiftSensor,Payload.data()+PayloadLocation,sizeof(NumberSwiftSensor));
      PayloadLocation += sizeof(NumberSwiftSensor);
      memcpy(&NumberAms5915Sensor,Payload.data()+PayloadLocation,sizeof(NumberAms5915Sensor));
      PayloadLocation += sizeof(NumberAms5915Sensor);
      memcpy(&NumberSbusSensor,Payload.data()+PayloadLocation,sizeof(NumberSbusSensor));
      PayloadLocation += sizeof(NumberSbusSensor);
      memcpy(&NumberAnalogSensor,Payload.data()+PayloadLocation,sizeof(NumberAnalogSensor));
      PayloadLocation += sizeof(NumberAnalogSensor);
      // resize data buffers
      if (AcquireInternalData & 0x01) {
        SensorData_.Time_us.resize(1);
        SensorNodes_.Time_us.resize(1);
      }
      if (AcquireInternalData & 0x02) {
        SensorData_.InternalMpu9250.resize(1);
        SensorNodes_.InternalMpu9250.resize(1);
      }
      if (AcquireInternalData & 0x04) {
        SensorData_.InternalBme280.resize(1);
        SensorNodes_.InternalBme280.resize(1);
      }
      if (AcquireInternalData & 0x08) {
        SensorData_.InputVoltage_V.resize(1);
        SensorNodes_.input_volts.resize(1);
      }
      if (AcquireInternalData & 0x10) {
        SensorData_.RegulatedVoltage_V.resize(1);
        SensorNodes_.reg_volts.resize(1);
      }
      SensorData_.PwmVoltage_V.resize(NumberPwmVoltageSensor);
      SensorData_.SbusVoltage_V.resize(NumberSbusVoltageSensor);
      SensorData_.Mpu9250.resize(NumberMpu9250Sensor);
      SensorData_.Bme280.resize(NumberBme280Sensor);
      SensorData_.uBlox.resize(NumberuBloxSensor);
      SensorData_.Swift.resize(NumberSwiftSensor);
      SensorData_.Ams5915.resize(NumberAms5915Sensor);
      SensorData_.Sbus.resize(NumberSbusSensor);
      SensorData_.Analog.resize(NumberAnalogSensor);
      if ( SensorNodes_.pwm_volts.size() < NumberPwmVoltageSensor ) {
        cout << "WARNING: RESIZING pwm_volts size to: "<< (int)NumberPwmVoltageSensor << endl;
        SensorNodes_.pwm_volts.resize(NumberPwmVoltageSensor);
      }
      if ( SensorNodes_.sbus_volts.size() < NumberSbusVoltageSensor ) {
        cout << "WARNING: RESIZING sbus_volts size to: "<< (int)NumberSbusVoltageSensor << endl;
        SensorNodes_.sbus_volts.resize(NumberSbusVoltageSensor);
      }
      if ( SensorNodes_.Mpu9250.size() < NumberMpu9250Sensor ) {
        cout << "WARNING: RESIZING Mpu9250 size to: "<< (int)NumberMpu9250Sensor << endl;
        SensorNodes_.Mpu9250.resize(NumberMpu9250Sensor);
      }
      if ( SensorNodes_.Bme280.size() < NumberBme280Sensor ) {
        cout << "WARNING: RESIZING Bme280 size to: "<< (int)NumberBme280Sensor << endl;
        SensorNodes_.Bme280.resize(NumberBme280Sensor);
      }
      if ( SensorNodes_.uBlox.size() < NumberuBloxSensor ) {
        cout << "WARNING: RESIZING uBlox size to: "<< (int)NumberuBloxSensor << endl;
        SensorNodes_.uBlox.resize(NumberuBloxSensor);
      }
      if ( SensorNodes_.Swift.size() < NumberSwiftSensor ) {
        cout << "WARNING: RESIZING Swift size to: "<< (int)NumberSwiftSensor << endl;
        SensorNodes_.Swift.resize(NumberSwiftSensor);
      }
      if ( SensorNodes_.Ams5915.size() < NumberAms5915Sensor ) {
        cout << "WARNING: RESIZING Ams5915 size to: "<< (int)NumberAms5915Sensor << endl;
        SensorNodes_.Ams5915.resize(NumberAms5915Sensor);
      }
      if ( SensorNodes_.Sbus.size() < NumberSbusSensor ) {
        cout << "WARNING: RESIZING Sbus size to: "<< (int)NumberSbusSensor << endl;
        SensorNodes_.Sbus.resize(NumberSbusSensor);
      }
      if ( SensorNodes_.Analog.size() < NumberAnalogSensor ) {
        cout << "WARNING: RESIZING Analog size to: "<< (int)NumberAnalogSensor << endl;
        SensorNodes_.Analog.resize(NumberAnalogSensor);
      }

      // sensor data
      memcpy(SensorData_.Time_us.data(),Payload.data()+PayloadLocation,SensorData_.Time_us.size()*sizeof(SensorData_.Time_us[0]));
      PayloadLocation += SensorData_.Time_us.size()*sizeof(SensorData_.Time_us[0]);
      memcpy(SensorData_.InternalMpu9250.data(),Payload.data()+PayloadLocation,SensorData_.InternalMpu9250.size()*sizeof(InternalMpu9250SensorData));
      PayloadLocation += SensorData_.InternalMpu9250.size()*sizeof(InternalMpu9250SensorData);
      memcpy(SensorData_.InternalBme280.data(),Payload.data()+PayloadLocation,SensorData_.InternalBme280.size()*sizeof(InternalBme280SensorData));
      PayloadLocation += SensorData_.InternalBme280.size()*sizeof(InternalBme280SensorData);
      memcpy(SensorData_.InputVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.InputVoltage_V.size()*sizeof(SensorData_.InputVoltage_V[0]));
      PayloadLocation += SensorData_.InputVoltage_V.size()*sizeof(SensorData_.InputVoltage_V[0]);
      memcpy(SensorData_.RegulatedVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.RegulatedVoltage_V.size()*sizeof(SensorData_.RegulatedVoltage_V[0]));
      PayloadLocation += SensorData_.RegulatedVoltage_V.size()*sizeof(SensorData_.RegulatedVoltage_V[0]);
      memcpy(SensorData_.PwmVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.PwmVoltage_V.size()*sizeof(SensorData_.PwmVoltage_V[0]));
      PayloadLocation += SensorData_.PwmVoltage_V.size()*sizeof(SensorData_.PwmVoltage_V[0]);
      memcpy(SensorData_.SbusVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.SbusVoltage_V.size()*sizeof(SensorData_.SbusVoltage_V[0]));
      PayloadLocation += SensorData_.SbusVoltage_V.size()*sizeof(SensorData_.SbusVoltage_V[0]);
      memcpy(SensorData_.Mpu9250.data(),Payload.data()+PayloadLocation,SensorData_.Mpu9250.size()*sizeof(Mpu9250SensorData));
      PayloadLocation += SensorData_.Mpu9250.size()*sizeof(Mpu9250SensorData);
      memcpy(SensorData_.Bme280.data(),Payload.data()+PayloadLocation,SensorData_.Bme280.size()*sizeof(Bme280SensorData));
      PayloadLocation += SensorData_.Bme280.size()*sizeof(Bme280SensorData);
      memcpy(SensorData_.uBlox.data(),Payload.data()+PayloadLocation,SensorData_.uBlox.size()*sizeof(uBloxSensorData));
      PayloadLocation += SensorData_.uBlox.size()*sizeof(uBloxSensorData);
      memcpy(SensorData_.Swift.data(),Payload.data()+PayloadLocation,SensorData_.Swift.size()*sizeof(SwiftSensorData));
      PayloadLocation += SensorData_.Swift.size()*sizeof(SwiftSensorData);
      memcpy(SensorData_.Ams5915.data(),Payload.data()+PayloadLocation,SensorData_.Ams5915.size()*sizeof(Ams5915SensorData));
      PayloadLocation += SensorData_.Ams5915.size()*sizeof(Ams5915SensorData);
      memcpy(SensorData_.Sbus.data(),Payload.data()+PayloadLocation,SensorData_.Sbus.size()*sizeof(SbusSensorData));
      PayloadLocation += SensorData_.Sbus.size()*sizeof(SbusSensorData);
      memcpy(SensorData_.Analog.data(),Payload.data()+PayloadLocation,SensorData_.Analog.size()*sizeof(AnalogSensorData));
      PayloadLocation += SensorData_.Analog.size()*sizeof(AnalogSensorData);

      if ( publish ) {
          // copy the incoming sensor data into the definition tree
          PublishSensors();
      }

      return true;
      freshdata = 1;
    }
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
	config_ack_msg.unpack(Payload.data(), Payload.size());
	printf("  received ack: %d ", config_ack_msg.ack_id);
	if ( id == config_ack_msg.ack_id and subid == config_ack_msg.ack_subid ) {
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
    message::config_analog_t msg;
    msg.output = Sensor["Output"].GetString();
    if ( Sensor.HasMember("Channel") ) {
      msg.channel = Sensor["Channel"].GetInt();
    } else {
        printf("ERROR: no analog channel specified");
    }
    msg.calibration[0] = 1.0;
    msg.calibration[1] = 0.0;
    msg.calibration[2] = 0.0;
    msg.calibration[3] = 0.0;
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
      msg.gain = sw["Threshold"].GetFloat();
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
      msg.gain = sw["Threshold"].GetFloat();
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
void FlightManagementUnit::ConfigureControlLaws(const rapidjson::Value& Config) {
  std::vector<uint8_t> Payload;
  if (Config.HasMember("Fmu")) {
    if (Config.HasMember(Config["Fmu"].GetString())) {
      rapidjson::StringBuffer FmuStringBuff;
      rapidjson::Writer<rapidjson::StringBuffer> FmuWriter(FmuStringBuff);
      const rapidjson::Value& Fmu = Config["Fmu"];
      Fmu.Accept(FmuWriter);
      std::string FmuString = FmuStringBuff.GetString();
      rapidjson::StringBuffer CntrlStringBuff;
      rapidjson::Writer<rapidjson::StringBuffer> CntrlWriter(CntrlStringBuff);
      const rapidjson::Value& Cntrl = Config[Config["Fmu"].GetString()];
      Cntrl.Accept(CntrlWriter);
      std::string CntrlString = CntrlStringBuff.GetString();
      std::string ConfigString = std::string("{\"Control\":{") + std::string("\"Fmu\":") + FmuString + std::string(",")
        + std::string("\"") + Config["Fmu"].GetString() + std::string("\":") + CntrlString + std::string("}}");
      for (size_t j=0; j < ConfigString.size(); j++) {
        Payload.push_back((uint8_t)ConfigString[j]);
      }
      SendMessage(Message::kConfigMesg, 0, Payload);
    }
  }
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
        msg.calibration[0] = 1.0;
        msg.calibration[1] = 0.0;
        msg.calibration[2] = 0.0;
        msg.calibration[3] = 0.0;
        if ( Effector.HasMember("Calibration") ) {
          if ( Effector["Calibration"].IsArray() and Effector["Calibration"].Size() <= 4 ) {
            for ( size_t i = 0; i < Effector["Calibration"].Size(); i++ ) {
              msg.calibration[i] = Effector["Calibration"][i].GetFloat();
            }
          } else {
            printf("ERROR: effector calibration incorrect\n");
          }
        }
        if ( Effector.HasMember("Safed-Command") ) {
          msg.safed_command = Effector["Safed-Command"].GetInt();
        } else if ( Effector["Type"] == "Motor" ){
          printf("ERROR: effector type motor without a safed-command\n");
        }
        msg.pack();
        SendMessage(msg.id, 0, msg.payload, msg.len);
        if ( ! WaitForAck(msg.id, 0, 1000) ) {
          printf("ERROR: effector command message failed ack!\n");
        }
      }
    } else {
      printf("ERROR: effector defined with no Type\n");
    }
  }
}

/* Registers sensor data with global definition tree */
void FlightManagementUnit::RegisterSensors(const rapidjson::Value& Config) {

  for (size_t i=0; i < SensorData_.Time_us.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Time",i);
    SensorNodes_.Time_us[i] = deftree.initElement(Path, "Flight management unit time, us", LOG_UINT64, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.InternalMpu9250.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"InternalMpu9250",i);
    SensorNodes_.InternalMpu9250[i].ax = deftree.initElement(Path+"/AccelX_mss", "Flight management unit MPU-9250 X accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].ay = deftree.initElement(Path+"/AccelY_mss", "Flight management unit MPU-9250 Y accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].az = deftree.initElement(Path+"/AccelZ_mss", "Flight management unit MPU-9250 Z accelerometer, corrected for installation rotation, m/s/s", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].p = deftree.initElement(Path+"/GyroX_rads", "Flight management unit MPU-9250 X gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].q = deftree.initElement(Path+"/GyroY_rads", "Flight management unit MPU-9250 Y gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].r = deftree.initElement(Path+"/GyroZ_rads", "Flight management unit MPU-9250 Z gyro, corrected for installation rotation, rad/s", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].hx = deftree.initElement(Path+"/MagX_uT", "Flight management unit MPU-9250 X magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].hy = deftree.initElement(Path+"/MagY_uT", "Flight management unit MPU-9250 Y magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].hz = deftree.initElement(Path+"/MagZ_uT", "Flight management unit MPU-9250 Z magnetometer, corrected for installation rotation, uT", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalMpu9250[i].temp = deftree.initElement(Path+"/Temperature_C", "Flight management unit MPU-9250 temperature, C", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.InternalBme280.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"InternalBme280",i);
    SensorNodes_.InternalBme280[i].press = deftree.initElement(Path+"/Pressure_Pa", "Flight management unit BME-280 static pressure, Pa", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalBme280[i].temp = deftree.initElement(Path+"/Temperature_C", "Flight management unit BME-280 temperature, C", LOG_FLOAT, LOG_NONE);
    SensorNodes_.InternalBme280[i].hum = deftree.initElement(Path+"/Humidity_RH", "Flight management unit BME-280 percent relative humidity", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.InputVoltage_V.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"InputVoltage",i);
    SensorNodes_.input_volts[i] = deftree.initElement(Path, "Flight management unit input voltage, V", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.RegulatedVoltage_V.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"RegulatedVoltage",i);
    SensorNodes_.reg_volts[i] = deftree.initElement(Path, "Flight management unit regulated voltage, V", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.PwmVoltage_V.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"PwmVoltage",i);
    SensorNodes_.pwm_volts[i] = deftree.initElement(Path, "Flight management unit PWM servo voltage, V", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.SbusVoltage_V.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"SbusVoltage",i);
    SensorNodes_.sbus_volts[i] = deftree.initElement(Path, "Flight management unit SBUS servo voltage, V", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.Mpu9250.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Mpu9250",i);
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
  }
  for (size_t i=0; i < SensorData_.Bme280.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Bme280",i);
    SensorNodes_.Bme280[i].status = deftree.initElement(Path+"/Status", "BME-280_" + to_string(i) + " read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
    SensorNodes_.Bme280[i].press = deftree.initElement(Path+"/Pressure_Pa", "BME-280_" + to_string(i) + " static pressure, Pa", LOG_FLOAT, LOG_NONE);
    SensorNodes_.Bme280[i].temp = deftree.initElement(Path+"/Temperature_C", "BME-280_" + to_string(i) + " temperature, C", LOG_FLOAT, LOG_NONE);
    SensorNodes_.Bme280[i].hum = deftree.initElement(Path+"/Humidity_RH", "BME-280_" + to_string(i) + " percent relative humidity", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.uBlox.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"uBlox",i);
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
  }
  for (size_t i=0; i < SensorData_.Swift.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Swift",i);
    SensorNodes_.Swift[i].Static.status = deftree.initElement(Path+"/Static/Status", "Swift_" + to_string(i) + " static pressure read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
    SensorNodes_.Swift[i].Static.press = deftree.initElement(Path+"/Static/Pressure_Pa", "Swift_" + to_string(i) + " static pressure, Pa", LOG_FLOAT, LOG_NONE);
    SensorNodes_.Swift[i].Static.temp = deftree.initElement(Path+"/Static/Temperature_C", "Swift_" + to_string(i) + " static pressure transducer temperature, C", LOG_FLOAT, LOG_NONE);
    SensorNodes_.Swift[i].Differential.status = deftree.initElement(Path+"/Differential/Status", "Swift_" + to_string(i) + " differential pressure read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
    SensorNodes_.Swift[i].Differential.press = deftree.initElement(Path+"/Differential/Pressure_Pa", "Swift_" + to_string(i) + " differential pressure, Pa", LOG_FLOAT, LOG_NONE);
    SensorNodes_.Swift[i].Differential.temp = deftree.initElement(Path+"/Differential/Temperature_C", "Swift_" + to_string(i) + " differential pressure transducer temperature, C", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.Ams5915.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Ams5915",i);
    SensorNodes_.Ams5915[i].status = deftree.initElement(Path+"/Status", "AMS-5915_" + to_string(i) + " read status, positive if a good sensor read", LOG_UINT8, LOG_NONE);
    SensorNodes_.Ams5915[i].press = deftree.initElement(Path+"/Pressure_Pa", "AMS-5915_" + to_string(i) + " pressure, Pa", LOG_FLOAT, LOG_NONE);
    SensorNodes_.Ams5915[i].temp = deftree.initElement(Path+"/Temperature_C", "AMS-5915_" + to_string(i) + " pressure transducer temperature, C", LOG_FLOAT, LOG_NONE);
  }
  for (size_t i=0; i < SensorData_.Sbus.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Sbus",i);
    SensorNodes_.Sbus[i].failsafe = deftree.initElement(Path+"/FailSafe", "SBUS_" + to_string(i) + " fail safe status", LOG_UINT8, LOG_NONE);
    SensorNodes_.Sbus[i].lost_frames = deftree.initElement(Path+"/LostFrames", "SBUS_" + to_string(i) + " number of lost frames", LOG_UINT64, LOG_NONE);
    for (size_t j=0; j < 16; j++) {
      SensorNodes_.Sbus[i].ch[j] = deftree.initElement(Path+"/Channels/"+to_string(j), "SBUS_" + to_string(i) + " channel" + to_string(j) + " normalized value", LOG_FLOAT, LOG_NONE);
    }
  }
  for (size_t i=0; i < SensorData_.Analog.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Analog",i);
    // SensorNodes_.Analog[i].volt = deftree.initElement(Path+"/Voltage_V", "Analog_" + to_string(i) + " measured voltage, V", LOG_FLOAT, LOG_NONE);
    SensorNodes_.Analog[i].val = deftree.initElement(Path+"/CalibratedValue", "Analog_" + to_string(i) + " calibrated value", LOG_FLOAT, LOG_NONE);
  }
}

/* Gets the sensor output name from JSON config given the "Type" and index */
std::string FlightManagementUnit::GetSensorOutputName(const rapidjson::Value& Config,std::string Key,size_t index) {
  size_t k=0;
  assert(Config.IsArray());
  for (size_t i=0; i < Config.Size(); i++) {
    const rapidjson::Value& Sensor = Config[i];
    if (Sensor.HasMember("Type")) {
      if (Sensor["Type"].GetString() == Key) {
        if (k == index) {
          return Sensor["Output"].GetString();
        }
        k++;
      }
      if (Sensor["Type"] == "Node") {
        const rapidjson::Value& Node = Sensor;
        if (Node.HasMember("Sensors")) {
          const rapidjson::Value& NodeSensors = Node["Sensors"];
          assert(NodeSensors.IsArray());
          for (size_t j=0; j < NodeSensors.Size(); j++) {
            const rapidjson::Value& NodeSensor = NodeSensors[j];
            if (NodeSensor.HasMember("Type")) {
              if (NodeSensor["Type"].GetString() == Key) {
                if (k == index) {
                  return NodeSensor["Output"].GetString();
                }
                k++;
              }
            }
          }
        }
      }
    }
  }
  return "";
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
    printf("receving msg: %d (size = %d)\n", *message, _bus->available());
    Payload->resize(_bus->available());
    _bus->read(Payload->data(),Payload->size());
    _bus->sendStatus(true);
    return true;
  } else {
    return false;
  }
}

void FlightManagementUnit::PublishSensors() {
  for (size_t i=0; i < SensorData_.Time_us.size(); i++) {
    // fixme: uint64_t
    SensorNodes_.Time_us[i]->setLong(SensorData_.Time_us[i]);
  }
  for (size_t i=0; i < SensorData_.InternalMpu9250.size(); i++) {
    SensorNodes_.InternalMpu9250[i].ax->setFloat(SensorData_.InternalMpu9250[i].AccelX_mss);
    SensorNodes_.InternalMpu9250[i].ay->setFloat(SensorData_.InternalMpu9250[i].AccelY_mss);
    SensorNodes_.InternalMpu9250[i].az->setFloat(SensorData_.InternalMpu9250[i].AccelZ_mss);
    SensorNodes_.InternalMpu9250[i].p->setFloat(SensorData_.InternalMpu9250[i].GyroX_rads);
    SensorNodes_.InternalMpu9250[i].q->setFloat(SensorData_.InternalMpu9250[i].GyroY_rads);
    SensorNodes_.InternalMpu9250[i].r->setFloat(SensorData_.InternalMpu9250[i].GyroZ_rads);
    SensorNodes_.InternalMpu9250[i].hx->setFloat(SensorData_.InternalMpu9250[i].MagX_uT);
    SensorNodes_.InternalMpu9250[i].hy->setFloat(SensorData_.InternalMpu9250[i].MagY_uT);
    SensorNodes_.InternalMpu9250[i].hz->setFloat(SensorData_.InternalMpu9250[i].MagZ_uT);
    SensorNodes_.InternalMpu9250[i].temp->setFloat(SensorData_.InternalMpu9250[i].Temperature_C);
  }
  for (size_t i=0; i < SensorData_.InternalBme280.size(); i++) {
    SensorNodes_.InternalBme280[i].press->setFloat(SensorData_.InternalBme280[i].Pressure_Pa);
    SensorNodes_.InternalBme280[i].temp->setFloat(SensorData_.InternalBme280[i].Temperature_C);
    SensorNodes_.InternalBme280[i].hum->setFloat(SensorData_.InternalBme280[i].Humidity_RH);
  }
  for (size_t i=0; i < SensorData_.InputVoltage_V.size(); i++) {
    SensorNodes_.input_volts[i]->setFloat(SensorData_.InputVoltage_V[i]);
  }
  for (size_t i=0; i < SensorData_.RegulatedVoltage_V.size(); i++) {
    SensorNodes_.reg_volts[i]->setFloat(SensorData_.RegulatedVoltage_V[i]);
  }
  for (size_t i=0; i < SensorData_.PwmVoltage_V.size(); i++) {
    SensorNodes_.pwm_volts[i]->setFloat(SensorData_.PwmVoltage_V[i]);
  }
  for (size_t i=0; i < SensorData_.SbusVoltage_V.size(); i++) {
    SensorNodes_.sbus_volts[i]->setFloat(SensorData_.SbusVoltage_V[i]);
  }
  for (size_t i=0; i < SensorData_.Mpu9250.size(); i++) {

    const float G = 9.807f;
    const float d2r = 3.14159265359f/180.0f;
    float accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
    float gyroScale = 2000.0f/32767.5f * d2r; // setting the gyro scale to 2000DPS

    SensorNodes_.Mpu9250[i].status->setInt(SensorData_.Mpu9250[i].status);
    SensorNodes_.Mpu9250[i].ax->setFloat((float) (SensorData_.Mpu9250[i].AccelX_ct) * accelScale);
    SensorNodes_.Mpu9250[i].ay->setFloat((float) (SensorData_.Mpu9250[i].AccelY_ct) * accelScale);
    SensorNodes_.Mpu9250[i].az->setFloat((float) (SensorData_.Mpu9250[i].AccelZ_ct) * accelScale);
    SensorNodes_.Mpu9250[i].p->setFloat((float) (SensorData_.Mpu9250[i].GyroX_ct) * gyroScale);
    SensorNodes_.Mpu9250[i].q->setFloat((float) (SensorData_.Mpu9250[i].GyroY_ct) * gyroScale);
    SensorNodes_.Mpu9250[i].r->setFloat((float) (SensorData_.Mpu9250[i].GyroZ_ct) * gyroScale);
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
