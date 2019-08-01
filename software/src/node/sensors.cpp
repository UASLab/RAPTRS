/*
sensors.cpp
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

#include "sensors.h"

static void HardFail(std::string message) {
  while (true) {
    Serial.println(message.c_str());
    delay(1000);
  }
}

/* update MPU9250 sensor configuration */
void Mpu9250Sensor::UpdateConfig(message::config_mpu9250_t *msg) {
  config_.UseSpi = msg->use_spi;
  if (config_.UseSpi) {
    config_.CsPin = msg->cs_pin;
    config_.MosiPin = msg->mosi_pin;
    config_.MisoPin = msg->miso_pin;
    config_.SckPin = msg->sck_pin;
    config_.Spi = msg->spi_bus;
  } else {
    config_.I2c = msg->i2c_bus;
    config_.Addr = msg->i2c_addr;
  }
  config_.Rotation(0,0) = msg->orientation[0];
  config_.Rotation(0,1) = msg->orientation[1];
  config_.Rotation(0,2) = msg->orientation[2];
  config_.Rotation(1,0) = msg->orientation[3];
  config_.Rotation(1,1) = msg->orientation[4];
  config_.Rotation(1,2) = msg->orientation[5];
  config_.Rotation(2,0) = msg->orientation[6];
  config_.Rotation(2,1) = msg->orientation[7];
  config_.Rotation(2,2) = msg->orientation[8];
  if ( msg->DLPF_bandwidth_hz > 0 ) {
    if (msg->DLPF_bandwidth_hz == 184) {
      config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_184HZ;
    } else if (msg->DLPF_bandwidth_hz == 92) {
      config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_92HZ;
    } else if (msg->DLPF_bandwidth_hz == 41) {
      config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_41HZ;
    } else if (msg->DLPF_bandwidth_hz == 20) {
      config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_20HZ;
    } else if (msg->DLPF_bandwidth_hz == 10) {
      config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_10HZ;
    } else if (msg->DLPF_bandwidth_hz == 5) {
      config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_5HZ;
    } else {
      HardFail("ERROR: Requested internal MPU9250 DLPF-Bandwidth is not an available option. Available options: 184Hz, 92Hz, 41Hz, 20Hz, 10Hz, 5Hz");
    }
  }
}

/* set the MPU9250 configuration */
void Mpu9250Sensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the MPU9250 configuration */
void Mpu9250Sensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* start communication and setup MPU9250 sensor */
void Mpu9250Sensor::Begin() {
  if (config_.UseSpi) {
    if (config_.Spi == 1) {
      _spi = &SPI1;
    } else {
      _spi = &SPI;
    }
    Mpu_ = new MPU9250(*_spi,config_.CsPin);
    _spi->setMOSI(config_.MosiPin);
    _spi->setMISO(config_.MisoPin);
    _spi->setSCK(config_.SckPin);
  } else {
    if (config_.I2c == 2) {
      _i2c = &Wire2;
    } else {
      _i2c = &Wire1;
    }
    Mpu_ = new MPU9250(*_i2c,config_.Addr);
  }
  while (Mpu_->begin() < 0) {
    Serial.print("ERROR: Failed to initialize MPU9250...");
    delay(400);
    Serial.println("\t retry...");
  }
  // set DLPF
  while (Mpu_->setDlpfBandwidth(config_.Bandwidth) < 0) {
    Serial.println("ERROR: Failed to set MPU9250 bandwidth.");
    delay(400);
    Serial.println("\t retry...");
  }
  // set SRD
  while (Mpu_->setSrd(config_.SRD) < 0) {
    Serial.println("ERROR: Failed to set MPU9250 SRD.");
    delay(400);
    Serial.println("\t retry...");
  }
  // enable data ready interrupt
  while (Mpu_->enableDataReadyInterrupt() < 0) {
    Serial.println("ERROR: Failed to enable MPU9250 data ready interrupt.");
    delay(400);
    Serial.println("\t retry...");
  }
}

/* get the MPU9250 sensor data */
int Mpu9250Sensor::ReadSensor() {
  Eigen::Matrix<float,3,1> Accel_Imu_mss;
  Eigen::Matrix<float,3,1> Gyro_Imu_rads;
  Eigen::Matrix<float,3,1> Mag_Imu_uT;

  status_ = Mpu_->readSensor();

  Accel_Imu_mss(0,0) = Mpu_->getAccelX_mss();
  Accel_Imu_mss(1,0) = Mpu_->getAccelY_mss();
  Accel_Imu_mss(2,0) = Mpu_->getAccelZ_mss();
  Gyro_Imu_rads(0,0) = Mpu_->getGyroX_rads();
  Gyro_Imu_rads(1,0) = Mpu_->getGyroY_rads();
  Gyro_Imu_rads(2,0) = Mpu_->getGyroZ_rads();
  Mag_Imu_uT(0,0) = Mpu_->getMagX_uT();
  Mag_Imu_uT(1,0) = Mpu_->getMagY_uT();
  Mag_Imu_uT(2,0) = Mpu_->getMagZ_uT();

  Accel_mss_ = config_.Rotation * Accel_Imu_mss;
  Gyro_rads_ = config_.Rotation * Gyro_Imu_rads;
  Mag_uT_ = config_.Rotation * Mag_Imu_uT;

  return status_;
}

/* get the MPU9250 sensor data */
void Mpu9250Sensor::UpdateMessage(message::data_mpu9250_short_t *msg) {
  // const float G = 9.807f;
  // const float d2r = 3.14159265359f/180.0f;
  // float accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
  // float gyroScale = 2000.0f/32767.5f * d2r; // setting the gyro scale to 2000DPS

  msg->ReadStatus = status_;
  msg->AccelX_mss = Accel_mss_(0,0);
  msg->AccelY_mss = Accel_mss_(1,0);
  msg->AccelZ_mss = Accel_mss_(2,0);
  msg->GyroX_rads = Gyro_rads_(0,0);
  msg->GyroY_rads = Gyro_rads_(1,0);
  msg->GyroZ_rads = Gyro_rads_(2,0);
}

/* free the MPU9250 sensor resources */
void Mpu9250Sensor::End() {
  delete Mpu_;
}

/* update the BME280 sensor configuration */
void Bme280Sensor::UpdateConfig(message::config_bme280_t *msg) {
  config_.UseSpi = msg->use_spi;
  if (config_.UseSpi) {
    config_.CsPin = msg->cs_pin;
    config_.MosiPin = msg->mosi_pin;
    config_.MisoPin = msg->miso_pin;
    config_.SckPin = msg->sck_pin;
    config_.Spi = msg->spi_bus;
  } else {
    config_.I2c = msg->i2c_bus;
    config_.Addr = msg->i2c_addr;
  }
}

/* set the BME280 sensor configuration */
void Bme280Sensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the BME280 sensor configuration */
void Bme280Sensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* begin communication with the BME280 sensor */
void Bme280Sensor::Begin() {
  if (config_.UseSpi) {
    if (config_.Spi == 1) {
      _spi = &SPI1;
    } else {
      _spi = &SPI;
    }
    Bme_ = new BME280(*_spi,config_.CsPin);
    _spi->setMOSI(config_.MosiPin);
    _spi->setMISO(config_.MisoPin);
    _spi->setSCK(config_.SckPin);
  } else {
    if (config_.I2c == 2) {
      _i2c = &Wire2;
    } else {
      _i2c = &Wire1;
    }
    Bme_ = new BME280(*_i2c,config_.Addr);
  }
  if (Bme_->begin() < 0) {
    while(1){
      Serial.println("ERROR: Failed to initialize BME280");
    }
  }
}

/* get data from the BME280 */
int Bme280Sensor::ReadSensor() {
  status_ = Bme_->readSensor();
  return status_;
}

/* get data from the BME280 */
void Bme280Sensor::UpdateMessage(message::data_bme280_t *msg) {
  msg->Pressure_Pa = Bme_->getPressure_Pa();
  msg->Temperature_C = Bme_->getTemperature_C();
  msg->Humidity_RH = Bme_->getHumidity_RH();
  msg->ReadStatus = status_;
}

/* free resources used by the BME280 sensor */
void Bme280Sensor::End() {
  delete Bme_;
}

/* update the uBlox configuration */
void uBloxSensor::UpdateConfig(message::config_ublox_t *msg) {
  if (msg->uart and msg->baud) {
    config_.Uart = msg->uart;
    config_.Baud = msg->baud;
  } else {
    HardFail("ERROR: uBlox port or baudrate missing from GPS configuration.");
  }
}

/* set the uBlox configuration */
void uBloxSensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the uBlox configuration */
void uBloxSensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* begin communication with the uBlox sensor */
void uBloxSensor::Begin() {
  switch (config_.Uart) {
    case 6: {
      ublox_ = new UBLOX(Serial6);
      break;
    }
    case 5: {
      ublox_ = new UBLOX(Serial5);
      break;
    }
    case 4: {
      ublox_ = new UBLOX(Serial4);
      break;
    }
    case 3: {
      ublox_ = new UBLOX(Serial3);
      break;
    }
    case 2: {
      ublox_ = new UBLOX(Serial2);
      break;
    }
    case 1: {
      ublox_ = new UBLOX(Serial1);
      break;
    }
  }
  ublox_->begin(config_.Baud);
}

/* update the private data structure with new uBlox data, if available */
int uBloxSensor::ReadSensor() {
  return ublox_->read(&uBloxData_);
}

/* update the private data structure with new uBlox data, if available */
void uBloxSensor::UpdateMessage(message::data_ublox_t *msg) {
  if (uBloxData_.fixType == 3) {
    msg->Fix = true;
  } else {
    msg->Fix = false;
  }
  msg->NumberSatellites = uBloxData_.numSV;
  msg->TOW = uBloxData_.iTOW;
  msg->Year = uBloxData_.utcYear;
  msg->Month = uBloxData_.utcMonth;
  msg->Day = uBloxData_.utcDay;
  msg->Hour = uBloxData_.utcHour;
  msg->Min = uBloxData_.utcMin;
  msg->Sec = uBloxData_.utcSec;
  msg->Latitude_rad = uBloxData_.lat*kD2R;
  msg->Longitude_rad = uBloxData_.lon*kD2R;
  msg->Altitude_m = uBloxData_.hMSL;
  msg->NorthVelocity_ms = uBloxData_.velN;
  msg->EastVelocity_ms = uBloxData_.velE;
  msg->DownVelocity_ms = uBloxData_.velD;
  msg->HorizontalAccuracy_m = uBloxData_.hAcc;
  msg->VerticalAccuracy_m = uBloxData_.vAcc;
  msg->VelocityAccuracy_ms = uBloxData_.sAcc;
  msg->pDOP = uBloxData_.pDOP;
}

/* free resources used by the uBlox sensor */
void uBloxSensor::End() {
  delete ublox_;
  switch (config_.Uart) {
    case 6: {
      Serial6.end();
      break;
    }
    case 5: {
      Serial5.end();
      break;
    }
    case 4: {
      Serial4.end();
      break;
    }
    case 3: {
      Serial3.end();
      break;
    }
    case 2: {
      Serial2.end();
      break;
    }
    case 1: {
      Serial1.end();
      break;
    }
  }
}

/* update the AMS5915 configuration */
void Ams5915Sensor::UpdateConfig(message::config_ams5915_t *msg) {
  config_.Addr = msg->i2c_addr;
  config_.I2c = msg->i2c_bus;
  if (msg->transducer == "AMS5915-0005-D") {
    config_.Transducer = AMS5915::AMS5915_0005_D;
  } else if (msg->transducer == "AMS5915-0010-D") {
    config_.Transducer = AMS5915::AMS5915_0010_D;
  } else if (msg->transducer == "AMS5915-0005-D-B") {
    config_.Transducer = AMS5915::AMS5915_0005_D_B;
  } else if (msg->transducer == "AMS5915-0010-D-B") {
    config_.Transducer = AMS5915::AMS5915_0010_D_B;
  } else if (msg->transducer == "AMS5915-0020-D") {
    config_.Transducer = AMS5915::AMS5915_0020_D;
  } else if (msg->transducer == "AMS5915-0050-D") {
    config_.Transducer = AMS5915::AMS5915_0050_D;
  } else if (msg->transducer == "AMS5915-0100-D") {
    config_.Transducer = AMS5915::AMS5915_0100_D;
  } else if (msg->transducer == "AMS5915-0020-D-B") {
    config_.Transducer = AMS5915::AMS5915_0020_D_B;
  } else if (msg->transducer == "AMS5915-0050-D-B") {
    config_.Transducer = AMS5915::AMS5915_0050_D_B;
  } else if (msg->transducer == "AMS5915-0100-D-B") {
    config_.Transducer = AMS5915::AMS5915_0100_D_B;
  } else if (msg->transducer == "AMS5915-0200-D") {
    config_.Transducer = AMS5915::AMS5915_0200_D;
  } else if (msg->transducer == "AMS5915-0350-D") {
    config_.Transducer = AMS5915::AMS5915_0350_D;
  } else if (msg->transducer == "AMS5915-1000-D") {
    config_.Transducer = AMS5915::AMS5915_1000_D;
  } else if (msg->transducer == "AMS5915-2000-D") {
    config_.Transducer = AMS5915::AMS5915_2000_D;
  } else if (msg->transducer == "AMS5915-4000-D") {
    config_.Transducer = AMS5915::AMS5915_4000_D;
  } else if (msg->transducer == "AMS5915-7000-D") {
    config_.Transducer = AMS5915::AMS5915_7000_D;
  } else if (msg->transducer == "AMS5915-10000-D") {
    config_.Transducer = AMS5915::AMS5915_10000_D;
  } else if (msg->transducer == "AMS5915-0200-D-B") {
    config_.Transducer = AMS5915::AMS5915_0200_D_B;
  } else if (msg->transducer == "AMS5915-0350-D-B") {
    config_.Transducer = AMS5915::AMS5915_0350_D_B;
  } else if (msg->transducer == "AMS5915-1000-D-B") {
    config_.Transducer = AMS5915::AMS5915_1000_D_B;
  } else if (msg->transducer == "AMS5915-1000-A") {
    config_.Transducer = AMS5915::AMS5915_1000_A;
  } else if (msg->transducer == "AMS5915-1200-B") {
    config_.Transducer = AMS5915::AMS5915_1200_B;
  } else {
    HardFail("ERROR: Requested AMS5915 transducer is not an available option.");
  }
}

/* set the AMS5915 configuration */
void Ams5915Sensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the AMS5915 configuration */
void Ams5915Sensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* start communication with the AMS5915 */
void Ams5915Sensor::Begin() {
  if (config_.I2c == 2) {
    _i2c = &Wire2;
  } else {
    _i2c = &Wire1;
  }
  ams_ = new AMS5915(*_i2c,config_.Addr,config_.Transducer);
  ams_->begin();
}

/* get data from the AMS5915 */
int Ams5915Sensor::ReadSensor() {
  status_ = ams_->readSensor();
  return status_;
}

/* get data from the AMS5915 */
void Ams5915Sensor::UpdateMessage(message::data_ams5915_t *msg) {
  msg->ReadStatus = status_;
  msg->Pressure_Pa = ams_->getPressure_Pa();
  msg->Temperature_C = ams_->getTemperature_C();
}

/* free resources used by the AMS5915 */
void Ams5915Sensor::End() {
  delete ams_;
}

/* update the Swift sensor configuration */
void SwiftSensor::UpdateConfig(message::config_swift_t *msg) {
  config_.Static.Addr = msg->static_i2c_addr;
  config_.Static.Transducer = AMS5915::AMS5915_1200_B;
  config_.Differential.Addr = msg->diff_i2c_addr;
  if (msg->diff_transducer == "AMS5915-0020-D") {
    config_.Differential.Transducer = AMS5915::AMS5915_0020_D;
  } else if (msg->diff_transducer == "AMS5915-0005-D") {
    config_.Differential.Transducer = AMS5915::AMS5915_0005_D;
  } else if (msg->diff_transducer == "AMS5915-0010-D") {
    config_.Differential.Transducer = AMS5915::AMS5915_0010_D;
  } else {
    HardFail("ERROR: Incompatible differential pressure transducer in pitot configuration.");
  }
  StaticAms.SetConfig(config_.Static);
  DiffAms.SetConfig(config_.Differential);
  config_.Static.I2c = msg->i2c_bus;
  config_.Differential.I2c = msg->i2c_bus;
}

/* set the Swift sensor configuration */
void SwiftSensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
  StaticAms.SetConfig(config_.Static);
  DiffAms.SetConfig(config_.Differential);
}

/* get the Swift sensor configuration */
void SwiftSensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* start communication with the Swift sensor */
void SwiftSensor::Begin() {
  StaticAms.Begin();
  DiffAms.Begin();
}

/* get data from the Swift sensor */
int SwiftSensor::ReadSensor() {
  StaticStatus_ = StaticAms.ReadSensor();
  DiffStatus_ = DiffAms.ReadSensor();
  if ( StaticStatus_ < 0 or DiffStatus_ < 0 ) {
    return 0;
  } else {
    return 1;
  }
}

/* get data from the Swift sensor */
void SwiftSensor::UpdateMessage(message::data_swift_t *msg) {
  // this is a slightly odd way to extract data from the individual
  // ams sensors, but C++ class privacy ...
  message::data_ams5915_t ams_msg;
  StaticAms.UpdateMessage(&ams_msg);
  msg->static_ReadStatus = ams_msg.ReadStatus;
  msg->static_Pressure_Pa = ams_msg.Pressure_Pa;
  msg->static_Temperature_C = ams_msg.Temperature_C;
  DiffAms.UpdateMessage(&ams_msg);
  msg->diff_ReadStatus = ams_msg.ReadStatus;
  msg->diff_Pressure_Pa = ams_msg.Pressure_Pa;
  msg->diff_Temperature_C = ams_msg.Temperature_C;
}

/* free resources used by the Swift sensor */
void SwiftSensor::End(){
  StaticAms.End();
  DiffAms.End();
}

/* update the SBUS receiver configuration */
void SbusSensor::UpdateConfig() {
  // pass
}

/* set the SBUS receiver configuration */
void SbusSensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the SBUS receiver configuration */
void SbusSensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* begin communication with the SBUS receiver */
void SbusSensor::Begin() {
  sbus_ = new SBUS(kSbusUart);
  sbus_->begin();
}

/* update the private data structure with SBUS receiver data, if available */
int SbusSensor::ReadSensor() {
  return sbus_->readCal(&channels_[0],&failsafe_,&lostframes_);
}

/* update the private data structure with SBUS receiver data, if available */
void SbusSensor::UpdateMessage(message::data_sbus_t *msg) {
  for (size_t i=0; i < 16; i++) {
    msg->channels[i] = channels_[i];
  }
  msg->FailSafe = (bool)failsafe_;
  msg->LostFrames = lostframes_;
}

/* free resources used by the SBUS receiver */
void SbusSensor::End() {
  delete sbus_;
  kSbusUart.end();
}

/* update the analog sensor configuration */
void AnalogSensor::UpdateConfig(message::config_analog_t *msg) {
  config_.Channel = msg->channel;
  for (int i = 0; i < message::max_calibration; i++) {
    if ( !isnanf(msg->calibration[i]) ) {
      config_.Calibration.push_back(msg->calibration[i]);
    }
  }
}

/* set the analog sensor configuration */
void AnalogSensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the analog sensor configuration */
void AnalogSensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* begin communication with the analog sensors */
void AnalogSensor::Begin() {
  analogReadResolution(kAnalogReadResolution);
}

/* get analog sensor data */
int AnalogSensor::ReadSensor() {
  uint8_t pin;
  if ( config_.DirectPin > 0 ) {
    pin = config_.DirectPin;
  } else {
    pin = kAnalogPins[config_.Channel];
  }
  Voltage_V_ = ((float)analogRead(pin))*3.3f/(powf(2,kAnalogReadResolution)-1.0f);
}

/* get analog sensor data */
void AnalogSensor::UpdateMessage(message::data_analog_t *msg) {
  msg->calibrated_value = PolyVal(config_.Calibration, Voltage_V_);
}

/* free resources used by the analog sensors */
void AnalogSensor::End() {}

/* updates the sensor configuration */
bool AircraftSensors::UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload) {
  delayMicroseconds(10);
  Serial.println("Updating sensor configuration...");
  if ( id == message::config_basic_id ) {
    message::config_basic_t msg;
    msg.unpack(Payload->data(), Payload->size());
    if ( msg.sensor == message::sensor_type::pwm_voltage ) {
      Serial.println("PwmVoltage");
      if (AcquirePwmVoltageData_) {
	HardFail("ERROR: Pwm voltage already initialized.");
      }
      AcquirePwmVoltageData_ = true;
      AnalogSensor::Config config;
      config.DirectPin = kPwmVoltagePin;
      config.Calibration.clear();
      config.Calibration.push_back(kEffectorVoltageScale);
      config.Calibration.push_back(0.0);
      classes_.Analog.push_back(AnalogSensor());
      classes_.Analog.back().SetConfig(config);
      return true;
    } else if (msg.sensor == message::sensor_type::sbus_voltage ) {
      Serial.println("SbusVoltage");
      if (AcquireSbusVoltageData_) {
	HardFail("ERROR: Sbus voltage already initialized.");
      }
      AcquireSbusVoltageData_ = true;
      AnalogSensor::Config config;
      config.DirectPin = kSbusVoltagePin;
      config.Calibration.clear();
      config.Calibration.push_back(kEffectorVoltageScale);
      config.Calibration.push_back(0.0);
      classes_.Analog.push_back(AnalogSensor());
      classes_.Analog.back().SetConfig(config);
      return true;
    } else if ( msg.sensor == message::sensor_type::sbus ) {
      Serial.println("Sbus");
      classes_.Sbus.push_back(SbusSensor());
      return true;
    }
  } else if ( id == message::config_mpu9250_id ) {
    Serial.println("Mpu9250");
    message::config_mpu9250_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Mpu9250.push_back(Mpu9250Sensor());
    classes_.Mpu9250.back().UpdateConfig(&msg);
    return true;
  } else if ( id == message::config_bme280_id ) {
    Serial.println("Bme280");
    message::config_bme280_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Bme280.push_back(Bme280Sensor());
    classes_.Bme280.back().UpdateConfig(&msg);
    return true;
  } else if ( id  == message::config_ublox_id ) {
    Serial.println("uBlox");
    message::config_ublox_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.uBlox.push_back(uBloxSensor());
    classes_.uBlox.back().UpdateConfig(&msg);
    return true;
  } else if ( id == message::config_swift_id ) {
    Serial.println("Swift");
    message::config_swift_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Swift.push_back(SwiftSensor());
    classes_.Swift.back().UpdateConfig(&msg);
    return true;
  } else if ( id == message::config_ams5915_id ) {
    Serial.println("Ams5915");
    message::config_ams5915_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Ams5915.push_back(Ams5915Sensor());
    classes_.Ams5915.back().UpdateConfig(&msg);
    return true;
  } else if ( id == message::config_analog_id ) {
    Serial.println("Analog");
    message::config_analog_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Analog.push_back(AnalogSensor());
    classes_.Analog.back().UpdateConfig(&msg);
    return true;
  }
  return false;
}

/* begin communication with all sensors */
void AircraftSensors::Begin() {
  Serial.println("Initializing sensors...");
  // set analog read resolution
  analogReadResolution(kAnalogReadResolution);
  // set all SPI pins high to deselect all of the sensors
  Serial.println("\tSetting Pins for MPU9250...");
  for (size_t i=0; i < classes_.Mpu9250.size(); i++) {
    Mpu9250Sensor::Config TempConfig;
    classes_.Mpu9250[i].GetConfig(&TempConfig);
    if (TempConfig.UseSpi) {
      pinMode(TempConfig.CsPin,OUTPUT);
      digitalWrite(TempConfig.CsPin,HIGH);
    }
  }

  Serial.println("\tSetting Pins for BME280...");
  for (size_t i=0; i < classes_.Bme280.size(); i++) {
    Bme280Sensor::Config TempConfig;
    classes_.Bme280[i].GetConfig(&TempConfig);
    if (TempConfig.UseSpi) {
      pinMode(TempConfig.CsPin,OUTPUT);
      digitalWrite(TempConfig.CsPin,HIGH);
    }
  }
  // begin all sensors

  Serial.print("\tMpu9250: ");
  // data_.Mpu9250.resize(classes_.Mpu9250.size());
  for (size_t i=0; i < classes_.Mpu9250.size(); i++) {
    classes_.Mpu9250[i].Begin();
  }
  Serial.println(classes_.Mpu9250.size());

  Serial.print("\tBme280: ");
  // data_.Bme280.resize(classes_.Bme280.size());
  for (size_t i=0; i < classes_.Bme280.size(); i++) {
    classes_.Bme280[i].Begin();
  }
  Serial.println(classes_.Bme280.size());

  Serial.print("\tuBlox: ");
  // data_.uBlox.resize(classes_.uBlox.size());
  for (size_t i=0; i < classes_.uBlox.size(); i++) {
    classes_.uBlox[i].Begin();
  }
  Serial.println(classes_.uBlox.size());

  Serial.print("\tSwift: ");
  // data_.Swift.resize(classes_.Swift.size());
  for (size_t i=0; i < classes_.Swift.size(); i++) {
    classes_.Swift[i].Begin();
  }
  Serial.println(classes_.Swift.size());

  Serial.print("\tAms5915: ");
  // data_.Ams5915.resize(classes_.Ams5915.size());
  for (size_t i=0; i < classes_.Ams5915.size(); i++) {
    classes_.Ams5915[i].Begin();
  }
  Serial.println(classes_.Ams5915.size());

  Serial.print("\tSbus: ");
  // data_.Sbus.resize(classes_.Sbus.size());
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].Begin();
  }
  Serial.println(classes_.Sbus.size());

  Serial.print("\tAnalog: ");
  // data_.Analog.resize(classes_.Analog.size());
  for (size_t i=0; i < classes_.Analog.size(); i++) {
    classes_.Analog[i].Begin();
  }
  Serial.println(classes_.Analog.size());

  Serial.print("\tPwm voltage: ");
  Serial.println(AcquirePwmVoltageData_);

  Serial.print("\tSbus voltage: ");
  Serial.println(AcquireSbusVoltageData_);

  Serial.println("done!");
}

/* read all synchronous sensors and store in data struct */
void AircraftSensors::ReadSyncSensors() {
  ResetI2cBus1_ = false;
  ResetI2cBus2_ = false;
  for (size_t i=0; i < classes_.Mpu9250.size(); i++) {
    int8_t status;
    status = classes_.Mpu9250[i].ReadSensor();
    if (status < 0) {
      Mpu9250Sensor::Config TempConfig;
      classes_.Mpu9250[i].GetConfig(&TempConfig);
      if (!TempConfig.UseSpi) {
        if (TempConfig.I2c == 2) {
          ResetI2cBus2_ = true;
        } else {
          ResetI2cBus1_ = true;
        }
      }
    }
  }
  for (size_t i=0; i < classes_.Bme280.size(); i++) {
    int8_t status;
    status = classes_.Bme280[i].ReadSensor();
    if (status < 0) {
      Bme280Sensor::Config TempConfig;
      classes_.Bme280[i].GetConfig(&TempConfig);
      if (!TempConfig.UseSpi) {
        if (TempConfig.I2c == 2) {
          ResetI2cBus2_ = true;
        } else {
          ResetI2cBus1_ = true;
        }
      }
    }
  }
  for (size_t i=0; i < classes_.Swift.size(); i++) {
    int8_t status;
    status = classes_.Swift[i].ReadSensor();
    if (status < 0) {
      SwiftSensor::Config TempConfig;
      classes_.Swift[i].GetConfig(&TempConfig);
      if (TempConfig.Static.I2c == 2) {
        ResetI2cBus2_ = true;
      } else {
        ResetI2cBus1_ = true;
      }
    }
  }
  for (size_t i=0; i < classes_.Ams5915.size(); i++) {
    int8_t status;
    status = classes_.Ams5915[i].ReadSensor();
    if (status < 0) {
      Ams5915Sensor::Config TempConfig;
      classes_.Ams5915[i].GetConfig(&TempConfig);
      if (TempConfig.I2c == 2) {
        ResetI2cBus2_ = true;
      } else {
        ResetI2cBus1_ = true;
      }
    }
  }
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].ReadSensor();
  }
  for (size_t i=0; i < classes_.Analog.size(); i++) {
    classes_.Analog[i].ReadSensor();
  }
  if (ResetI2cBus1_) {
    Wire1.resetBus();
  }
  if (ResetI2cBus2_) {
    Wire2.resetBus();
  }
}

/* read all asynchronous sensors and store in data struct */
void AircraftSensors::ReadAsyncSensors() {
  for (size_t i=0; i < classes_.uBlox.size(); i++) {
    classes_.uBlox[i].ReadSensor();
  }
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].ReadSensor();
  }
}

/* get data struct */
// void AircraftSensors::GetData(Data *DataPtr) {
//   *DataPtr = data_;
// }

static void add_msg(std::vector<uint8_t> *Buffer, uint8_t id, uint8_t index, uint8_t len, uint8_t *payload) {
  if ( Buffer->size() + len > Buffer->capacity() ) {
    Buffer->resize(Buffer->size() + len);
    Serial.print("Notice: increasing Buffer size to: ");
    Serial.println(Buffer->capacity());
  }
  Buffer->push_back(id);
  Buffer->push_back(index);     // I think I can get rid of this byte (fixme?)
  Buffer->push_back(len);
  Buffer->insert(Buffer->end(), payload, payload + len);
}

/* get data buffer */
void AircraftSensors::MakeCompoundMessage(std::vector<uint8_t> *Buffer, std::vector<uint8_t> *SizeBuffer) {
  Buffer->clear();
  {
    message::data_mpu9250_short_t msg;
    for ( size_t i = 0; i < classes_.Mpu9250.size(); i++ ) {
      classes_.Mpu9250[i].UpdateMessage(&msg);
      msg.pack();
      add_msg(Buffer, msg.id, i, msg.len, msg.payload);
    }
  }
  {
    message::data_bme280_t msg;
    for ( size_t i = 0; i < classes_.Bme280.size(); i++ ) {
      classes_.Bme280[i].UpdateMessage(&msg);
      msg.pack();
      add_msg(Buffer, msg.id, i, msg.len, msg.payload);
    }
  }
  {
    message::data_ublox_t msg;
    for ( size_t i = 0; i < classes_.uBlox.size(); i++ ) {
      classes_.uBlox[i].UpdateMessage(&msg);
      msg.pack();
      add_msg(Buffer, msg.id, i, msg.len, msg.payload);
    }
  }
  {
    message::data_swift_t msg;
    for ( size_t i = 0; i < classes_.Swift.size(); i++ ) {
      classes_.Swift[i].UpdateMessage(&msg);
      msg.pack();
      add_msg(Buffer, msg.id, i, msg.len, msg.payload);
    }
  }
  {
    message::data_sbus_t msg;
    for ( size_t i = 0; i < classes_.Sbus.size(); i++ ) {
      classes_.Sbus[i].UpdateMessage(&msg);
      msg.pack();
      add_msg(Buffer, msg.id, i, msg.len, msg.payload);
    }
  }
  {
    message::data_ams5915_t msg;
    for ( size_t i = 0; i < classes_.Ams5915.size(); i++ ) {
      classes_.Ams5915[i].UpdateMessage(&msg);
      msg.pack();
      add_msg(Buffer, msg.id, i, msg.len, msg.payload);
    }
  }
  {
    message::data_analog_t msg;
    for ( size_t i = 0; i < classes_.Analog.size(); i++ ) {
      classes_.Analog[i].UpdateMessage(&msg);
      msg.pack();
      add_msg(Buffer, msg.id, i, msg.len, msg.payload);
    }
  }
  SizeBuffer->clear();
  SizeBuffer->resize(sizeof(uint16_t));
  *(uint16_t *)(SizeBuffer->data()) = Buffer->size();
}

/* free all sensor resources and reset sensor vectors and states */
void AircraftSensors::End() {
  Serial.print("Freeing sensors...");
  // free up sensor resources
  for (size_t i=0; i < classes_.Mpu9250.size(); i++) {
    classes_.Mpu9250[i].End();
  }
  for (size_t i=0; i < classes_.Bme280.size(); i++) {
    classes_.Bme280[i].End();
  }
  for (size_t i=0; i < classes_.uBlox.size(); i++) {
    classes_.uBlox[i].End();
  }
  for (size_t i=0; i < classes_.Swift.size(); i++) {
    classes_.Swift[i].End();
  }
  for (size_t i=0; i < classes_.Ams5915.size(); i++) {
    classes_.Ams5915[i].End();
  }
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].End();
  }
  for (size_t i=0; i < classes_.Analog.size(); i++) {
    classes_.Analog[i].End();
  }
  // clear class and data vectors
  classes_.Mpu9250.clear();
  classes_.Bme280.clear();
  classes_.uBlox.clear();
  classes_.Swift.clear();
  classes_.Ams5915.clear();
  classes_.Sbus.clear();
  classes_.Analog.clear();
  AcquirePwmVoltageData_ = false;
  AcquireSbusVoltageData_ = false;
  Serial.println("done!");
}
