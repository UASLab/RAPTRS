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
int Mpu9250Sensor::GetData(Data *DataPtr) {
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

  const float G = 9.807f;
  const float d2r = 3.14159265359f/180.0f;
  float accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
  float gyroScale = 2000.0f/32767.5f * d2r; // setting the gyro scale to 2000DPS

  data_.ReadStatus = status_;
  data_.AccelX_ct = (int) (Accel_mss_(0,0) / accelScale);
  data_.AccelY_ct = (int) (Accel_mss_(1,0) / accelScale);
  data_.AccelZ_ct = (int) (Accel_mss_(2,0) / accelScale);
  data_.GyroX_ct = (int) (Gyro_rads_(0,0) / gyroScale);
  data_.GyroY_ct = (int) (Gyro_rads_(1,0) / gyroScale);
  data_.GyroZ_ct = (int) (Gyro_rads_(2,0) / gyroScale);
  // data_.MagX_ct = Mag_uT(0,0);
  // data_.MagY_ct = Mag_uT(1,0);
  // data_.MagZ_ct = Mag_uT(2,0);
  // data_.Temperature_ct = Mpu_->getTemperature_C();
  *DataPtr = data_;
  return status_;
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
int Bme280Sensor::GetData(Data *DataPtr) {
  status_ = Bme_->readSensor();
  data_.Pressure_Pa = Bme_->getPressure_Pa();
  data_.Temperature_C = Bme_->getTemperature_C();
  data_.Humidity_RH = Bme_->getHumidity_RH();
  data_.ReadStatus = status_;
  *DataPtr = data_;
  return status_;
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
void uBloxSensor::UpdateData() {
  if (ublox_->read(&uBloxData_)) {
    if (uBloxData_.fixType == 3) {
      data_.Fix = true;
    } else {
      data_.Fix = false;
    }
    data_.NumberSatellites = uBloxData_.numSV;
    data_.TOW = uBloxData_.iTOW;
    data_.Year = uBloxData_.utcYear;
    data_.Month = uBloxData_.utcMonth;
    data_.Day = uBloxData_.utcDay;
    data_.Hour = uBloxData_.utcHour;
    data_.Min = uBloxData_.utcMin;
    data_.Sec = uBloxData_.utcSec;
    data_.Latitude_rad = uBloxData_.lat*kD2R;
    data_.Longitude_rad = uBloxData_.lon*kD2R;
    data_.Altitude_m = uBloxData_.hMSL;
    data_.NorthVelocity_ms = uBloxData_.velN;
    data_.EastVelocity_ms = uBloxData_.velE;
    data_.DownVelocity_ms = uBloxData_.velD;
    data_.HorizontalAccuracy_m = uBloxData_.hAcc;
    data_.VerticalAccuracy_m = uBloxData_.vAcc;
    data_.VelocityAccuracy_ms = uBloxData_.sAcc;
    data_.pDOP = uBloxData_.pDOP;
  }
}

/* get the private data structure */
void uBloxSensor::GetData(Data *DataPtr) {
  *DataPtr = data_;
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
int Ams5915Sensor::GetData(Data *DataPtr) {
  status_ = ams_->readSensor();
  data_.Pressure_Pa = ams_->getPressure_Pa();
  data_.Temperature_C = ams_->getTemperature_C();
  data_.ReadStatus = status_;
  *DataPtr = data_;
  return status_;
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
int SwiftSensor::GetData(Data *DataPtr) {
  StaticStatus_ = StaticAms.GetData(&data_.Static);
  DiffStatus_ = DiffAms.GetData(&data_.Differential);
  *DataPtr = data_;
  if ((StaticStatus_ < 0)||(DiffStatus_ < 0)) {
    return -1;
  } else {
    return 1;
  }
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
void SbusSensor::UpdateData() {
  if (sbus_->readCal(&channels_[0],&failsafe_,&lostframes_)) {
    for (size_t i=0; i < 16; i++) {
      data_.Channels[i] = channels_[i];
    }
    data_.FailSafe = (bool)failsafe_;
    data_.LostFrames = lostframes_;
  }
}

/* get the SBUS private data structure */
void SbusSensor::GetData(Data *DataPtr) {
  *DataPtr = data_;
}

/* free resources used by the SBUS receiver */
void SbusSensor::End() {
  delete sbus_;
  kSbusUart.end();
}

/* update the analog sensor configuration */
void AnalogSensor::UpdateConfig(message::config_analog_t *msg) {
  config_.Channel = msg->channel;
  int last_coeff = message::max_calibration - 1;
  for (int i=0; i < message::max_calibration; i++) {
    if ( fabs(msg->calibration[i]) > 0.000001 ) {
      last_coeff = i;
    }
  }
  for (int i = 0; i < last_coeff; i++) {
    config_.Calibration.push_back(msg->calibration[i]);
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
void AnalogSensor::GetData(Data *DataPtr) {
  Voltage_V_ = ((float)analogRead(kAnalogPins[config_.Channel]))*3.3f/(powf(2,kAnalogReadResolution)-1.0f);
  data_.CalibratedValue = PolyVal(config_.Calibration, Voltage_V_);
  *DataPtr = data_;
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
      return true;
    } else if (msg.sensor == message::sensor_type::sbus_voltage ) {
      Serial.println("SbusVoltage");
      if (AcquireSbusVoltageData_) {
	HardFail("ERROR: Sbus voltage already initialized.");
      }
      AcquireSbusVoltageData_ = true;
      return true;
    } else if ( msg.sensor == message::sensor_type::sbus ) {
      Serial.println("Sbus");
      classes_.Sbus.push_back(SbusSensor());
      data_.Sbus.resize(classes_.Sbus.size());
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
  data_.Mpu9250.resize(classes_.Mpu9250.size());
  for (size_t i=0; i < classes_.Mpu9250.size(); i++) {
    classes_.Mpu9250[i].Begin();
  }
  Serial.println(classes_.Mpu9250.size());

  Serial.print("\tBme280: ");
  data_.Bme280.resize(classes_.Bme280.size());
  for (size_t i=0; i < classes_.Bme280.size(); i++) {
    classes_.Bme280[i].Begin();
  }
  Serial.println(classes_.Bme280.size());

  Serial.print("\tuBlox: ");
  data_.uBlox.resize(classes_.uBlox.size());
  for (size_t i=0; i < classes_.uBlox.size(); i++) {
    classes_.uBlox[i].Begin();
  }
  Serial.println(classes_.uBlox.size());

  Serial.print("\tSwift: ");
  data_.Swift.resize(classes_.Swift.size());
  for (size_t i=0; i < classes_.Swift.size(); i++) {
    classes_.Swift[i].Begin();
  }
  Serial.println(classes_.Swift.size());

  Serial.print("\tAms5915: ");
  data_.Ams5915.resize(classes_.Ams5915.size());
  for (size_t i=0; i < classes_.Ams5915.size(); i++) {
    classes_.Ams5915[i].Begin();
  }
  Serial.println(classes_.Ams5915.size());

  Serial.print("\tSbus: ");
  data_.Sbus.resize(classes_.Sbus.size());
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].Begin();
  }
  Serial.println(classes_.Sbus.size());

  Serial.print("\tSbus voltage: ");
  if (AcquireSbusVoltageData_) {
    data_.SbusVoltage_V.resize(1);
  }
  Serial.println(AcquireSbusVoltageData_);

  Serial.print("\tAnalog: ");
  data_.Analog.resize(classes_.Analog.size());
  for (size_t i=0; i < classes_.Analog.size(); i++) {
    classes_.Analog[i].Begin();
  }
  Serial.println(classes_.Analog.size());

  Serial.print("\tPwm voltage: ");
  if (AcquirePwmVoltageData_) {
    data_.PwmVoltage_V.resize(1);
  }
  Serial.println(AcquirePwmVoltageData_);

  Serial.println("done!");
}

/* read all synchronous sensors and store in data struct */
void AircraftSensors::ReadSyncSensors() {
  ResetI2cBus1_ = false;
  ResetI2cBus2_ = false;
  if (AcquirePwmVoltageData_) {
    data_.PwmVoltage_V[0] = ((float)analogRead(kPwmVoltagePin))*3.3f/(powf(2,kAnalogReadResolution)-1.0f)*kEffectorVoltageScale;
  }
  if (AcquireSbusVoltageData_) {
    data_.SbusVoltage_V[0] = ((float)analogRead(kSbusVoltagePin))*3.3f/(powf(2,kAnalogReadResolution)-1.0f)*kEffectorVoltageScale;
  }
  for (size_t i=0; i < classes_.Mpu9250.size(); i++) {
    int8_t status;
    status = classes_.Mpu9250[i].GetData(&data_.Mpu9250[i]);
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
    status = classes_.Bme280[i].GetData(&data_.Bme280[i]);
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
  for (size_t i=0; i < classes_.uBlox.size(); i++) {
    classes_.uBlox[i].GetData(&data_.uBlox[i]);
  }
  for (size_t i=0; i < classes_.Swift.size(); i++) {
    int8_t status;
    status = classes_.Swift[i].GetData(&data_.Swift[i]);
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
    status = classes_.Ams5915[i].GetData(&data_.Ams5915[i]);
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
    classes_.Sbus[i].GetData(&data_.Sbus[i]);
  }
  for (size_t i=0; i < classes_.Analog.size(); i++) {
    classes_.Analog[i].GetData(&data_.Analog[i]);
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
    classes_.uBlox[i].UpdateData();
  }
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].UpdateData();
  }
}

/* get data struct */
void AircraftSensors::GetData(Data *DataPtr) {
  *DataPtr = data_;
}

/* get data buffer */
void AircraftSensors::GetDataBuffer(std::vector<uint8_t> *Buffer) {
  size_t BufferLocation = 0;
  Buffer->resize(sizeof(data_.PwmVoltage_V[0])*data_.PwmVoltage_V.size()+
    sizeof(data_.SbusVoltage_V[0])*data_.SbusVoltage_V.size()+
    sizeof(Mpu9250Sensor::Data)*data_.Mpu9250.size()+
    sizeof(Bme280Sensor::Data)*data_.Bme280.size()+
    sizeof(uBloxSensor::Data)*data_.uBlox.size()+
    sizeof(SwiftSensor::Data)*data_.Swift.size()+
    sizeof(SbusSensor::Data)*data_.Sbus.size()+
    sizeof(Ams5915Sensor::Data)*data_.Ams5915.size()+
    sizeof(AnalogSensor::Data)*data_.Analog.size());
  // sensor data
  memcpy(Buffer->data()+BufferLocation,data_.PwmVoltage_V.data(),data_.PwmVoltage_V.size()*sizeof(data_.PwmVoltage_V[0]));
  BufferLocation+=data_.PwmVoltage_V.size()*sizeof(data_.PwmVoltage_V[0]);
  memcpy(Buffer->data()+BufferLocation,data_.SbusVoltage_V.data(),data_.SbusVoltage_V.size()*sizeof(data_.SbusVoltage_V[0]));
  BufferLocation+=data_.SbusVoltage_V.size()*sizeof(data_.SbusVoltage_V[0]);
  memcpy(Buffer->data()+BufferLocation,data_.Mpu9250.data(),data_.Mpu9250.size()*sizeof(Mpu9250Sensor::Data));
  BufferLocation+=data_.Mpu9250.size()*sizeof(Mpu9250Sensor::Data);
  memcpy(Buffer->data()+BufferLocation,data_.Bme280.data(),data_.Bme280.size()*sizeof(Bme280Sensor::Data));
  BufferLocation+=data_.Bme280.size()*sizeof(Bme280Sensor::Data);
  memcpy(Buffer->data()+BufferLocation,data_.uBlox.data(),data_.uBlox.size()*sizeof(uBloxSensor::Data));
  BufferLocation+=data_.uBlox.size()*sizeof(uBloxSensor::Data);
  memcpy(Buffer->data()+BufferLocation,data_.Swift.data(),data_.Swift.size()*sizeof(SwiftSensor::Data));
  BufferLocation+=data_.Swift.size()*sizeof(SwiftSensor::Data);
  memcpy(Buffer->data()+BufferLocation,data_.Ams5915.data(),data_.Ams5915.size()*sizeof(Ams5915Sensor::Data));
  BufferLocation+=data_.Ams5915.size()*sizeof(Ams5915Sensor::Data);
  memcpy(Buffer->data()+BufferLocation,data_.Sbus.data(),data_.Sbus.size()*sizeof(SbusSensor::Data));
  BufferLocation+=data_.Sbus.size()*sizeof(SbusSensor::Data);
  memcpy(Buffer->data()+BufferLocation,data_.Analog.data(),data_.Analog.size()*sizeof(AnalogSensor::Data));
  BufferLocation+=data_.Analog.size()*sizeof(AnalogSensor::Data);
}

/* get meta data buffer */
void AircraftSensors::GetMetaDataBuffer(std::vector<uint8_t> *Buffer) {
  size_t BufferLocation = 0;
  Buffer->resize(SerializedDataMetadataSize);
  // meta data
  uint8_t AcquireInternalData,NumberMpu9250Sensor,NumberBme280Sensor,NumberuBloxSensor,NumberSwiftSensor,NumberAms5915Sensor,NumberSbusSensor,NumberAnalogSensor;
  AcquireInternalData = 0x00;
  if (AcquirePwmVoltageData_) {
    AcquireInternalData |=  0x20;
  }
  if (AcquireSbusVoltageData_) {
    AcquireInternalData |=  0x40;
  }
  NumberMpu9250Sensor = data_.Mpu9250.size();
  NumberBme280Sensor = data_.Bme280.size();
  NumberuBloxSensor = data_.uBlox.size();
  NumberSwiftSensor = data_.Swift.size();
  NumberAms5915Sensor = data_.Ams5915.size();
  NumberSbusSensor = data_.Sbus.size();
  NumberAnalogSensor = data_.Analog.size();
  memcpy(Buffer->data()+BufferLocation,&AcquireInternalData,sizeof(AcquireInternalData));
  BufferLocation+=sizeof(AcquireInternalData);
  memcpy(Buffer->data()+BufferLocation,&NumberMpu9250Sensor,sizeof(NumberMpu9250Sensor));
  BufferLocation+=sizeof(NumberMpu9250Sensor);
  memcpy(Buffer->data()+BufferLocation,&NumberBme280Sensor,sizeof(NumberBme280Sensor));
  BufferLocation+=sizeof(NumberBme280Sensor);
  memcpy(Buffer->data()+BufferLocation,&NumberuBloxSensor,sizeof(NumberuBloxSensor));
  BufferLocation+=sizeof(NumberuBloxSensor);
  memcpy(Buffer->data()+BufferLocation,&NumberSwiftSensor,sizeof(NumberSwiftSensor));
  BufferLocation+=sizeof(NumberSwiftSensor);
  memcpy(Buffer->data()+BufferLocation,&NumberAms5915Sensor,sizeof(NumberAms5915Sensor));
  BufferLocation+=sizeof(NumberAms5915Sensor);
  memcpy(Buffer->data()+BufferLocation,&NumberSbusSensor,sizeof(NumberSbusSensor));
  BufferLocation+=sizeof(NumberSbusSensor);
  memcpy(Buffer->data()+BufferLocation,&NumberAnalogSensor,sizeof(NumberAnalogSensor));
  BufferLocation+=sizeof(NumberAnalogSensor);
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
  data_.PwmVoltage_V.clear();
  data_.SbusVoltage_V.clear();
  data_.Mpu9250.clear();
  data_.Bme280.clear();
  data_.uBlox.clear();
  data_.Swift.clear();
  data_.Ams5915.clear();
  data_.Sbus.clear();
  data_.Analog.clear();
  // reset data acquisition flags
  AcquirePwmVoltageData_ = false;
  AcquireSbusVoltageData_ = false;
  Serial.println("done!");
}
