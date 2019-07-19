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

/* update internal MPU9250 sensor configuration */
void InternalMpu9250Sensor::UpdateConfig(message_config_mpu9250_t *msg, std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  if (msg->SRD > 0) {
    config_.SRD = msg->SRD;
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
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelX_mss",&data_.AccelX_mss);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelY_mss",&data_.AccelY_mss);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelZ_mss",&data_.AccelZ_mss);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroX_rads",&data_.GyroX_rads);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroY_rads",&data_.GyroY_rads);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroZ_rads",&data_.GyroZ_rads);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/MagX_uT",&data_.MagX_uT);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/MagY_uT",&data_.MagY_uT);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/MagZ_uT",&data_.MagZ_uT);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Temperature_C",&data_.Temperature_C);
}

/* set the internal MPU9250 configuration */
void InternalMpu9250Sensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the internal MPU9250 configuration */
void InternalMpu9250Sensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* start communication and setup internal MPU9250 sensor */
void InternalMpu9250Sensor::Begin() {
  // MPU9250 object on hardware defined CS Pin
  Mpu_ = new MPU9250(SPI,kMpu9250CsPin);
  // initialize communication with MPU9250
  if (Mpu_->begin() < 0) {
    while(1){
      Serial.println("ERROR: Failed to initialize internal MPU9250.");
    }
  }
  // set DLPF
  if (Mpu_->setDlpfBandwidth(config_.Bandwidth) < 0) {
    while(1){
      Serial.println("ERROR: Failed to set internal MPU9250 bandwidth.");
    }
  }
  // set SRD
  if (Mpu_->setSrd(config_.SRD) < 0) {
    while(1){
      Serial.println("ERROR: Failed to set internal MPU9250 SRD.");
    }
  }
  // enable data ready interrupt
  if (Mpu_->enableDataReadyInterrupt() < 0) {
    while(1){
      Serial.println("ERROR: Failed to enable internal MPU9250 data ready interrupt.");
    }
  }
  // setting the internal MPU9250 orientation relative to the FMU
  for (size_t m=0; m < 3; m++) {
    for (size_t n=0; n < 3; n++) {
      config_.Orientation(m,n) = kMpu9250Orientation[m][n];
    }
  }
}

/* get the internal MPU9250 sensor data */
void InternalMpu9250Sensor::GetData(Data *DataPtr) {
  Eigen::Matrix<float,3,1>Accel_Imu_mss;
  Eigen::Matrix<float,3,1>Gyro_Imu_rads;
  Eigen::Matrix<float,3,1>Mag_Imu_uT;
  Mpu_->readSensor();
  Accel_Imu_mss(0,0) = Mpu_->getAccelX_mss();
  Accel_Imu_mss(1,0) = Mpu_->getAccelY_mss();
  Accel_Imu_mss(2,0) = Mpu_->getAccelZ_mss();
  Gyro_Imu_rads(0,0) = Mpu_->getGyroX_rads();
  Gyro_Imu_rads(1,0) = Mpu_->getGyroY_rads();
  Gyro_Imu_rads(2,0) = Mpu_->getGyroZ_rads();
  Mag_Imu_uT(0,0) = Mpu_->getMagX_uT();
  Mag_Imu_uT(1,0) = Mpu_->getMagY_uT();
  Mag_Imu_uT(2,0) = Mpu_->getMagZ_uT();

  Accel_mss_ = config_.Rotation * config_.Orientation * Accel_Imu_mss;
  Gyro_rads_ = config_.Rotation * config_.Orientation * Gyro_Imu_rads;
  Mag_uT_ = config_.Rotation * config_.Orientation * Mag_Imu_uT;

  data_.AccelX_mss = Accel_mss_(0,0);
  data_.AccelY_mss = Accel_mss_(1,0);
  data_.AccelZ_mss = Accel_mss_(2,0);
  data_.GyroX_rads = Gyro_rads_(0,0);
  data_.GyroY_rads = Gyro_rads_(1,0);
  data_.GyroZ_rads = Gyro_rads_(2,0);
  data_.MagX_uT = Mag_uT_(0,0);
  data_.MagY_uT = Mag_uT_(1,0);
  data_.MagZ_uT = Mag_uT_(2,0);
  data_.Temperature_C = Mpu_->getTemperature_C();
  *DataPtr = data_;
}

/* free the internal MPU9250 sensor resources */
void InternalMpu9250Sensor::End() {
  delete Mpu_;
}

/* update MPU9250 sensor configuration */
void Mpu9250Sensor::UpdateConfig(message_config_mpu9250_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  config_.UseSpi = msg->use_spi;
  if (config_.UseSpi) {
    config_.CsPin = msg->cs_pin;
  } else {
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
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Status",(int8_t*)&data_.ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelX_mss",&Accel_mss_(0,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelY_mss",&Accel_mss_(1,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelZ_mss",&Accel_mss_(2,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroX_rads",&Gyro_rads_(0,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroY_rads",&Gyro_rads_(1,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroZ_rads",&Gyro_rads_(2,0));
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
    Mpu_ = new MPU9250(SPI,config_.CsPin);
  } else {
    Mpu_ = new MPU9250(Wire1,config_.Addr);
  }
  if (Mpu_->begin() < 0) {
    while(1){
      Serial.println("ERROR: Failed to initialize MPU9250.");
    }
  }
  // set DLPF
  if (Mpu_->setDlpfBandwidth(config_.Bandwidth) < 0) {
    while(1){
      Serial.println("ERROR: Failed to set MPU9250 bandwidth.");
    }
  }
  // set SRD
  if (Mpu_->setSrd(config_.SRD) < 0) {
    while(1){
      Serial.println("ERROR: Failed to set MPU9250 SRD.");
    }
  }
  // enable data ready interrupt
  if (Mpu_->enableDataReadyInterrupt() < 0) {
    while(1){
      Serial.println("ERROR: Failed to enable MPU9250 data ready interrupt.");
    }
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

/* update the internal BME280 sensor configuration */
void InternalBme280Sensor::UpdateConfig(std::string Output,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Pressure_Pa",&data_.Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Temperature_C);
  DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Humidity_RH",&data_.Humidity_RH);
}

/* set the internal BME280 sensor configuration */
void InternalBme280Sensor::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the internal BME280 sensor configuration */
void InternalBme280Sensor::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* begin communication with the internal BME280 sensor */
void InternalBme280Sensor::Begin() {
  Bme_ = new BME280(SPI,kBme280CsPin);
  if (Bme_->begin() < 0) {
    while(1){
      Serial.println("ERROR: Failed to initialize internal BME280");
    }
  }
}

/* get data from the internal BME280 sensor */
void InternalBme280Sensor::GetData(Data *DataPtr) {
  Bme_->readSensor();
  data_.Pressure_Pa = Bme_->getPressure_Pa();
  data_.Temperature_C = Bme_->getTemperature_C();
  data_.Humidity_RH = Bme_->getHumidity_RH();
  *DataPtr = data_;
}

/* free resources used by the internal BME280 sensor */
void InternalBme280Sensor::End() {
  delete Bme_;
}

/* update the BME280 sensor configuration */
void Bme280Sensor::UpdateConfig(message_config_bme280_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  config_.UseSpi = msg->use_spi;
  if (config_.UseSpi) {
    config_.CsPin = msg->cs_pin;
  } else {
    config_.Addr = msg->i2c_addr;
  }
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Status",(int8_t*)&data_.ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Pressure_Pa",&data_.Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Temperature_C",&data_.Temperature_C);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Humidity_RH",&data_.Humidity_RH);
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
    Bme_ = new BME280(SPI,config_.CsPin);
  } else {
    Bme_ = new BME280(Wire1,config_.Addr);
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
void uBloxSensor::UpdateConfig(message_config_ublox_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  if (msg->uart and msg->baud) {
    config_.Uart = msg->uart;
    config_.Baud = msg->baud;
  } else {
    HardFail("ERROR: uBlox port or baudrate missing from GPS configuration.");
  }
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Fix",(uint8_t*)&data_.Fix);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/NumberSatellites",&data_.NumberSatellites);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/TOW",&data_.TOW);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Year",&data_.Year);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Month",&data_.Month);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Day",&data_.Day);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Hour",&data_.Hour);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Minute",&data_.Min);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Second",&data_.Sec);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Latitude_rad",&data_.Longitude_rad);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Longitude_rad",&data_.Latitude_rad);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Altitude_m",&data_.Altitude_m);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/NorthVelocity_ms",&data_.NorthVelocity_ms);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/EastVelocity_ms",&data_.EastVelocity_ms);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/DownVelocity_ms",&data_.DownVelocity_ms);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/HorizontalAccuracy_m",&data_.HorizontalAccuracy_m);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/VerticalAccuracy_m",&data_.VerticalAccuracy_m);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/VelocityAccuracy_ms",&data_.VelocityAccuracy_ms);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/pDOP",&data_.pDOP);
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
void Ams5915Sensor::UpdateConfig(message_config_ams5915_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  config_.Addr = msg->i2c_addr;
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
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Status",(int8_t*)&data_.ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Pressure_Pa",&data_.Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Temperature_C",&data_.Temperature_C);
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
  ams_ = new AMS5915(Wire1,config_.Addr,config_.Transducer);
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
void SwiftSensor::UpdateConfig(message_config_swift_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
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
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Static/Status",(int8_t*)&data_.Static.ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Static/Pressure_Pa",&data_.Static.Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Static/Temperature_C",&data_.Static.Temperature_C);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Differential/Status",(int8_t*)&data_.Differential.ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Differential/Pressure_Pa",&data_.Differential.Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Differential/Temperature_C",&data_.Differential.Temperature_C);
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
void SbusSensor::UpdateConfig(std::string output, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  DefinitionTreePtr->InitMember(RootPath+"/"+output+"/FailSafe",(uint8_t*)&data_.FailSafe);
  DefinitionTreePtr->InitMember(RootPath+"/"+output+"/LostFrames",&data_.LostFrames);
  for (size_t j=0; j < 16; j++) {
    DefinitionTreePtr->InitMember(RootPath+"/"+output+"/Channels/" + std::to_string(j),&data_.Channels[j]);
  }
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
void AnalogSensor::UpdateConfig(message_config_analog_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  config_.Channel = msg->channel;
  int last_coeff = 3;
  for (int i=0; i < message_max_calibration; i++) {
    if ( fabs(msg->calibration[i]) > 0.000001 ) {
      last_coeff = i;
    }
  }
  for (int i=0; i < message_max_calibration; i++) {
    config_.Calibration.push_back(msg->calibration[i]);
  }
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Voltage_V",(uint8_t*)&Voltage_V_);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/CalibratedValue",&data_.CalibratedValue);
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

SensorNodes::SensorNodes(uint8_t address) {
  config_.BfsAddr = address;
}

/* updates the node configuration */
void SensorNodes::UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  Serial.print("SensorNodes Parsing: ");
  if ( node_ == NULL ) {
    // create a new node
    node_ = new Node(kBfsPort,config_.BfsAddr,kBfsRate);
    // start communication with node
    node_->Begin();
  }
  // send config messages
  node_->SetConfigurationMode();
  node_->Configure(id, Payload);
  // update local structures and definition tree to match
  if ( id == message_config_basic_id ) {
    message_config_basic_t msg;
    msg.unpack(Payload->data(), Payload->size());
    if (msg.device == config_basic::pwm_voltage) {
      data_.PwmVoltage_V.resize(1);
      DefinitionTreePtr->InitMember(RootPath + "/" + msg.output, &data_.PwmVoltage_V[0]);
    } else if (msg.device == config_basic::sbus_voltage) {
      data_.SbusVoltage_V.resize(1);
      DefinitionTreePtr->InitMember(RootPath + "/" + msg.output, &data_.SbusVoltage_V[0]);
    } else if (msg.device == config_basic::sbus) {
      data_.Sbus.resize(data_.Sbus.size()+1);
      DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/FailSafe",(uint8_t*)&data_.Sbus.back().FailSafe);
      DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/LostFrames",&data_.Sbus.back().LostFrames);
      for (size_t j=0; j < 16; j++) {
        DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Channels/" + std::to_string(j),&data_.Sbus.back().Channels[j]);
      }
    }
  } else if ( id == message_config_mpu9250_id ) {
    message_config_mpu9250_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Mpu9250.resize(data_.Mpu9250.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Status",(int8_t*)&data_.Mpu9250.back().ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/AccelX_mss",&data_.Mpu9250.back().AccelX_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/AccelY_mss",&data_.Mpu9250.back().AccelY_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/AccelZ_mss",&data_.Mpu9250.back().AccelZ_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/GyroX_rads",&data_.Mpu9250.back().GyroX_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/GyroY_rads",&data_.Mpu9250.back().GyroY_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/GyroZ_rads",&data_.Mpu9250.back().GyroZ_ct);
  } else if ( id == message_config_bme280_id ) {
    message_config_bme280_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Bme280.resize(data_.Bme280.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Status",(int8_t*)&data_.Bme280.back().ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Pressure_Pa",&data_.Bme280.back().Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Temperature_C",&data_.Bme280.back().Temperature_C);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Humidity_RH",&data_.Bme280.back().Humidity_RH);
  } else if ( id == message_config_ublox_id ) {
    message_config_ublox_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.uBlox.resize(data_.uBlox.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Fix",(uint8_t*)&data_.uBlox.back().Fix);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/NumberSatellites",&data_.uBlox.back().NumberSatellites);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/TOW",&data_.uBlox.back().TOW);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Year",&data_.uBlox.back().Year);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Month",&data_.uBlox.back().Month);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Day",&data_.uBlox.back().Day);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Hour",&data_.uBlox.back().Hour);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Minute",&data_.uBlox.back().Min);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Second",&data_.uBlox.back().Sec);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Latitude_rad",&data_.uBlox.back().Latitude_rad);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Longitude_rad",&data_.uBlox.back().Longitude_rad);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Altitude_m",&data_.uBlox.back().Altitude_m);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/NorthVelocity_ms",&data_.uBlox.back().NorthVelocity_ms);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/EastVelocity_ms",&data_.uBlox.back().EastVelocity_ms);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/DownVelocity_ms",&data_.uBlox.back().DownVelocity_ms);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/HorizontalAccuracy_m",&data_.uBlox.back().HorizontalAccuracy_m);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/VerticalAccuracy_m",&data_.uBlox.back().VerticalAccuracy_m);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/VelocityAccuracy_ms",&data_.uBlox.back().VelocityAccuracy_ms);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/pDOP",&data_.uBlox.back().pDOP);
  } else if ( id == message_config_swift_id ) {
    message_config_swift_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Swift.resize(data_.Swift.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Static/Status",(int8_t*)&data_.Swift.back().Static.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Static/Pressure_Pa",&data_.Swift.back().Static.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Static/Temperature_C",&data_.Swift.back().Static.Temperature_C);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Differential/Status",(int8_t*)&data_.Swift.back().Differential.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Differential/Pressure_Pa",&data_.Swift.back().Differential.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Differential/Temperature_C",&data_.Swift.back().Differential.Temperature_C);
  } else if ( id == message_config_ams5915_id ) {
    message_config_ams5915_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Ams5915.resize(data_.Ams5915.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Status",(int8_t*)&data_.Ams5915.back().ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Pressure_Pa",&data_.Ams5915.back().Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Temperature_C",&data_.Ams5915.back().Temperature_C);
  } else if ( id == message_config_analog_id ) {
    message_config_analog_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Analog.resize(data_.Analog.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/CalibratedValue",&data_.Analog.back().CalibratedValue);
  }
}

/* set the node configuration */
void SensorNodes::SetConfig(const Config &ConfigRef) {
  config_ = ConfigRef;
}

/* get the node configuration */
void SensorNodes::GetConfig(Config *ConfigPtr) {
  *ConfigPtr = config_;
}

/* begin communication with nodes and configure */
void SensorNodes::Begin(Data *DataPtr) {
  node_->SetRunMode();
  while (1) {
    digitalWriteFast(kBfsInt1Pin,HIGH);
    if (node_->ReadSensorData()) {
      DataPtr->PwmVoltage_V.resize(node_->GetNumberPwmVoltageSensor());
      DataPtr->SbusVoltage_V.resize(node_->GetNumberSbusVoltageSensor());
      DataPtr->Mpu9250.resize(node_->GetNumberMpu9250Sensor());
      DataPtr->Bme280.resize(node_->GetNumberBme280Sensor());
      DataPtr->uBlox.resize(node_->GetNumberuBloxSensor());
      DataPtr->Swift.resize(node_->GetNumberSwiftSensor());
      DataPtr->Ams5915.resize(node_->GetNumberAms5915Sensor());
      DataPtr->Sbus.resize(node_->GetNumberSbusSensor());
      DataPtr->Analog.resize(node_->GetNumberAnalogSensor());
      break;
    }
    delay(5);
    digitalWriteFast(kBfsInt1Pin,LOW);
  }
}

/* get data from the nodes */
int SensorNodes::GetData(Data *DataPtr) {
  std::vector<uint8_t> SensorDataBuffer;
  size_t BufferLocation = 0;
  DataPtr->PwmVoltage_V.clear();
  DataPtr->SbusVoltage_V.clear();
  DataPtr->Mpu9250.clear();
  DataPtr->Bme280.clear();
  DataPtr->uBlox.clear();
  DataPtr->Swift.clear();
  DataPtr->Ams5915.clear();
  DataPtr->Sbus.clear();
  DataPtr->Analog.clear();
  if (node_->ReadSensorData()) {
    DataPtr->PwmVoltage_V.resize(node_->GetNumberPwmVoltageSensor());
    DataPtr->SbusVoltage_V.resize(node_->GetNumberSbusVoltageSensor());
    DataPtr->Mpu9250.resize(node_->GetNumberMpu9250Sensor());
    DataPtr->Bme280.resize(node_->GetNumberBme280Sensor());
    DataPtr->uBlox.resize(node_->GetNumberuBloxSensor());
    DataPtr->Swift.resize(node_->GetNumberSwiftSensor());
    DataPtr->Ams5915.resize(node_->GetNumberAms5915Sensor());
    DataPtr->Sbus.resize(node_->GetNumberSbusSensor());
    DataPtr->Analog.resize(node_->GetNumberAnalogSensor());
    node_->GetSensorDataBuffer(&SensorDataBuffer);
    // sensor data
    memcpy(DataPtr->PwmVoltage_V.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->PwmVoltage_V.size()*sizeof(DataPtr->PwmVoltage_V[0]));
    BufferLocation += DataPtr->PwmVoltage_V.size()*sizeof(DataPtr->PwmVoltage_V[0]);
    memcpy(DataPtr->SbusVoltage_V.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->SbusVoltage_V.size()*sizeof(DataPtr->SbusVoltage_V[0]));
    BufferLocation += DataPtr->SbusVoltage_V.size()*sizeof(DataPtr->SbusVoltage_V[0]);
    memcpy(DataPtr->Mpu9250.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->Mpu9250.size()*sizeof(Mpu9250Sensor::Data));
    BufferLocation += DataPtr->Mpu9250.size()*sizeof(Mpu9250Sensor::Data);
    memcpy(DataPtr->Bme280.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->Bme280.size()*sizeof(Bme280Sensor::Data));
    BufferLocation += DataPtr->Bme280.size()*sizeof(Bme280Sensor::Data);
    memcpy(DataPtr->uBlox.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->uBlox.size()*sizeof(uBloxSensor::Data));
    BufferLocation += DataPtr->uBlox.size()*sizeof(uBloxSensor::Data);
    memcpy(DataPtr->Swift.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->Swift.size()*sizeof(SwiftSensor::Data));
    BufferLocation += DataPtr->Swift.size()*sizeof(SwiftSensor::Data);
    memcpy(DataPtr->Ams5915.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->Ams5915.size()*sizeof(Ams5915Sensor::Data));
    BufferLocation += DataPtr->Ams5915.size()*sizeof(Ams5915Sensor::Data);
    memcpy(DataPtr->Sbus.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->Sbus.size()*sizeof(SbusSensor::Data));
    BufferLocation += DataPtr->Sbus.size()*sizeof(SbusSensor::Data);
    memcpy(DataPtr->Analog.data(),SensorDataBuffer.data()+BufferLocation,DataPtr->Analog.size()*sizeof(AnalogSensor::Data));
    BufferLocation += DataPtr->Analog.size()*sizeof(AnalogSensor::Data);
    return 1;
  } else {
    return -1;
  }
}

/* free resources from the nodes */
void SensorNodes::End() {
  delete node_;
}

/* updates the sensor configuration */
bool AircraftSensors::UpdateConfig(uint8_t id, uint8_t address, std::vector<uint8_t> *Payload, DefinitionTree *DefinitionTreePtr) {
  delayMicroseconds(10);
  Serial.print("Updating sensor configuration:");
  if ( address > 0 ) {
    // this message is relayed to a node
    int found = -1;
    for ( unsigned int i = 0; i < classes_.Nodes.size(); i++ ) {
      if ( classes_.Nodes[i].GetBfsAddr() == address ) {
        found = i;
      }
    }
    if ( found < 0 ) {
      found = classes_.Nodes.size();
      classes_.Nodes.push_back(SensorNodes(address));
      data_.Nodes.resize(classes_.Nodes.size());
    }
    classes_.Nodes[found].UpdateConfig(id, Payload, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message_config_basic_id ) {
    message_config_basic_t msg;
    msg.unpack(Payload->data(), Payload->size());
    if ( msg.device == config_basic::time ) {
      Serial.println("Time");
      if (AcquireTimeData_) {
	HardFail("ERROR: Time already initialized.");
      }
      AcquireTimeData_ = true;
      data_.Time_us.resize(1);
      std::string Output = msg.output;
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &data_.Time_us[0]);
      return true;
    } else if ( msg.device == config_basic::input_voltage ) {
      Serial.println("InputVoltage");
      if (AcquireInputVoltageData_) {
	HardFail("ERROR: Input voltage already initialized.");
      }
      AcquireInputVoltageData_ = true;
      data_.InputVoltage_V.resize(1);
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &data_.InputVoltage_V[0]);
      return true;
    } else if ( msg.device == config_basic::regulated_voltage ) {
      Serial.println("RegulatedVoltage");
      if (AcquireRegulatedVoltageData_) {
	HardFail("ERROR: Regulated voltage already initialized.");
      }
      AcquireRegulatedVoltageData_ = true;
      data_.RegulatedVoltage_V.resize(1);
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &data_.RegulatedVoltage_V[0]);
      return true;
    } else if ( msg.device == config_basic::pwm_voltage ) {
      Serial.println("PwmVoltage");
      if (AcquirePwmVoltageData_) {
	HardFail("ERROR: Pwm voltage already initialized.");
      }
      AcquirePwmVoltageData_ = true;
      data_.PwmVoltage_V.resize(1);
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &data_.PwmVoltage_V[0]);
      return true;
    } else if (msg.device == config_basic::sbus_voltage ) {
      Serial.println("SbusVoltage");
      if (AcquireSbusVoltageData_) {
	HardFail("ERROR: Sbus voltage already initialized.");
      }
      AcquireSbusVoltageData_ = true;
      data_.SbusVoltage_V.resize(1);
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &data_.SbusVoltage_V[0]);
      return true;
    } else if ( msg.device == config_basic::internal_bme280 ) {
      Serial.println("InternalBme280");
      if (classes_.InternalBme280.size() > 0) {
	HardFail("ERROR: Internal BME280 already initialized.");
      }
      InternalBme280Sensor TempSensor;
      classes_.InternalBme280.push_back(TempSensor);
      data_.InternalBme280.resize(classes_.InternalBme280.size());
      classes_.InternalBme280.back().UpdateConfig(msg.output,RootPath_,DefinitionTreePtr);
      return true;
    } else if ( msg.device == config_basic::sbus ) {
      Serial.println("Sbus");
      classes_.Sbus.push_back(SbusSensor());
      data_.Sbus.resize(classes_.Sbus.size());
      classes_.Sbus.back().UpdateConfig(msg.output, RootPath_, DefinitionTreePtr);
      return true;
    }
  } else if ( id == message_config_mpu9250_id ) {
    message_config_mpu9250_t msg;
    msg.unpack(Payload->data(), Payload->size());
    if ( msg.internal ) {
      Serial.println("InternalMpu9250");
      if (AcquireInternalMpu9250Data_) {
	HardFail("ERROR: Internal MPU9250 already initialized.");
      }
      AcquireInternalMpu9250Data_ = true;
      data_.InternalMpu9250.resize(1);
      classes_.InternalMpu9250.UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
      return true;      
    } else {
      Serial.println("Mpu9250");
      classes_.Mpu9250.push_back(Mpu9250Sensor());
      data_.Mpu9250.resize(classes_.Mpu9250.size());
      classes_.Mpu9250.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
      return true;
    }
  } else if ( id  == message_config_ublox_id ) {
    message_config_ublox_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.uBlox.push_back(uBloxSensor());
    data_.uBlox.resize(classes_.uBlox.size());
    classes_.uBlox.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message_config_swift_id ) {
    message_config_swift_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Swift.push_back(SwiftSensor());
    data_.Swift.resize(classes_.Swift.size());
    classes_.Swift.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message_config_bme280_id ) {
    message_config_bme280_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Bme280.push_back(Bme280Sensor());
    data_.Bme280.resize(classes_.Bme280.size());
    classes_.Bme280.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message_config_ams5915_id ) {
    message_config_ams5915_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Ams5915.push_back(Ams5915Sensor());
    data_.Ams5915.resize(classes_.Ams5915.size());
    classes_.Ams5915.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message_config_analog_id ) {
    message_config_analog_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Analog.push_back(AnalogSensor());
    data_.Analog.resize(classes_.Analog.size());
    classes_.Analog.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  }
  return false;
}

/* begin communication with all sensors */
void AircraftSensors::Begin() {
  Serial.println("Initializing FMU sensors...");
  // set analog read resolution
  analogReadResolution(kAnalogReadResolution);
  // set all SPI pins high to deselect all of the sensors
  pinMode(kMpu9250CsPin,OUTPUT);
  digitalWrite(kMpu9250CsPin,HIGH);
  pinMode(kBme280CsPin,OUTPUT);
  digitalWrite(kBme280CsPin,HIGH);

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
  Serial.print("\tTime: ");
  Serial.println(AcquireTimeData_);
  Serial.print("\tInput voltage: ");
  Serial.println(AcquireInputVoltageData_);
  Serial.print("\tRegulated voltage: ");
  Serial.println(AcquireRegulatedVoltageData_);

  Serial.print("\tInternal Mpu9250: ");
  classes_.InternalMpu9250.Begin();
  Serial.println(AcquireInternalMpu9250Data_);

  Serial.print("\tInternal Bme280: ");
  for (size_t i=0; i < classes_.InternalBme280.size(); i++) {
    classes_.InternalBme280[i].Begin();
  }
  Serial.println(classes_.InternalBme280.size());

  Serial.print("\tMpu9250: ");
  for (size_t i=0; i < classes_.Mpu9250.size(); i++) {
    classes_.Mpu9250[i].Begin();
  }
  Serial.println(classes_.Mpu9250.size());

  Serial.print("\tBme280: ");
  for (size_t i=0; i < classes_.Bme280.size(); i++) {
    classes_.Bme280[i].Begin();
  }
  Serial.println(classes_.Bme280.size());

  Serial.print("\tuBlox: ");
  for (size_t i=0; i < classes_.uBlox.size(); i++) {
    classes_.uBlox[i].Begin();
  }
  Serial.println(classes_.uBlox.size());

  Serial.print("\tSwift: ");
  for (size_t i=0; i < classes_.Swift.size(); i++) {
    classes_.Swift[i].Begin();
  }
  Serial.println(classes_.Swift.size());

  Serial.print("\tAms5915: ");
  for (size_t i=0; i < classes_.Ams5915.size(); i++) {
    classes_.Ams5915[i].Begin();
  }
  Serial.println(classes_.Ams5915.size());

  Serial.print("\tSbus: ");
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].Begin();
  }
  Serial.println(classes_.Sbus.size());
  Serial.print("\tSbus voltage: ");
  Serial.println(AcquireSbusVoltageData_);

  Serial.print("\tAnalog: ");
  for (size_t i=0; i < classes_.Analog.size(); i++) {
    classes_.Analog[i].Begin();
  }
  Serial.println(classes_.Analog.size());

  Serial.print("\tPwm voltage: ");
  Serial.println(AcquirePwmVoltageData_);


  Serial.println("Initializing Nodes...");
  for (size_t i=0; i < classes_.Nodes.size(); i++) {
    Serial.println("\tBegin().. ");
    classes_.Nodes[i].Begin(&data_.Nodes[i]);

    Serial.println("\tGetConfig().. ");
    SensorNodes::Config TempConfig;
    classes_.Nodes[i].GetConfig(&TempConfig);

    Serial.print("\tNode ");
    Serial.print(TempConfig.BfsAddr);
    Serial.println(":");

    Serial.print("\t\tPwm voltage: ");
    Serial.println(data_.Nodes[i].PwmVoltage_V.size());
    Serial.print("\t\tSbus voltage: ");
    Serial.println(data_.Nodes[i].SbusVoltage_V.size());
    Serial.print("\t\tMpu9250: ");
    Serial.println(data_.Nodes[i].Mpu9250.size());
    Serial.print("\t\tBme280: ");
    Serial.println(data_.Nodes[i].Bme280.size());
    Serial.print("\t\tuBlox: ");
    Serial.println(data_.Nodes[i].uBlox.size());
    Serial.print("\t\tSwift: ");
    Serial.println(data_.Nodes[i].Swift.size());
    Serial.print("\t\tAms5915: ");
    Serial.println(data_.Nodes[i].Ams5915.size());
    Serial.print("\t\tSbus: ");
    Serial.println(data_.Nodes[i].Sbus.size());
    Serial.print("\t\tAnalog: ");
    Serial.println(data_.Nodes[i].Analog.size());
  }
  Serial.println("done!");
}

/* read all synchronous sensors and store in data struct */
void AircraftSensors::ReadSyncSensors() {
  ResetI2cBus_ = false;
  ResetBfsBus_ = false;
  digitalWriteFast(kBfsInt1Pin,HIGH);
  digitalWriteFast(kBfsInt2Pin,LOW);
  if (AcquirePwmVoltageData_) {
    data_.PwmVoltage_V.resize(1);
  } else {
    data_.PwmVoltage_V.resize(0);
  }
  if (AcquireSbusVoltageData_) {
    data_.SbusVoltage_V.resize(1);
  } else {
    data_.SbusVoltage_V.resize(0);
  }
  data_.Mpu9250.resize(classes_.Mpu9250.size());
  data_.Bme280.resize(classes_.Bme280.size());
  data_.uBlox.resize(classes_.uBlox.size());
  data_.Swift.resize(classes_.Swift.size());
  data_.Ams5915.resize(classes_.Ams5915.size());
  data_.Sbus.resize(classes_.Sbus.size());
  data_.Analog.resize(classes_.Analog.size());
  if (AcquireTimeData_) {
    data_.Time_us[0] = micros_64();
  }
  if (AcquireInternalMpu9250Data_) {
    classes_.InternalMpu9250.GetData(&data_.InternalMpu9250[0]);
  }
  for (size_t i=0; i < classes_.InternalBme280.size(); i++) {
    classes_.InternalBme280[i].GetData(&data_.InternalBme280[i]);
  }
  if (AcquireInputVoltageData_) {
    data_.InputVoltage_V[0] = ((float)analogRead(kInputVoltagePin))*3.3f/(powf(2,kAnalogReadResolution)-1.0f)*kInputVoltageScale;
  }
  if (AcquireRegulatedVoltageData_) {
    data_.RegulatedVoltage_V[0] = ((float)analogRead(kRegulatedVoltagePin))*3.3f/(powf(2,kAnalogReadResolution)-1.0f)*kRegulatedVoltageScale;
  }
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
        ResetI2cBus_ = true;
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
        ResetI2cBus_ = true;
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
      ResetI2cBus_ = true;
    }
  }
  for (size_t i=0; i < classes_.Ams5915.size(); i++) {
    int8_t status;
    status = classes_.Ams5915[i].GetData(&data_.Ams5915[i]);
    if (status < 0) {
      ResetI2cBus_ = true;
    }
  }
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].GetData(&data_.Sbus[i]);
  }
  for (size_t i=0; i < classes_.Analog.size(); i++) {
    classes_.Analog[i].GetData(&data_.Analog[i]);
  }
  for (size_t i=0; i < classes_.Nodes.size(); i++) {
    int8_t status;
    status = classes_.Nodes[i].GetData(&data_.Nodes[i]);
    if (status < 0) {
      ResetBfsBus_ = true;
    }
  }
  for (size_t i=0; i < data_.Nodes.size(); i++) {
    for (size_t j=0; j < data_.Nodes[i].PwmVoltage_V.size(); j++) {
      data_.PwmVoltage_V.push_back(data_.Nodes[i].PwmVoltage_V[j]);
    }
    for (size_t j=0; j < data_.Nodes[i].SbusVoltage_V.size(); j++) {
      data_.SbusVoltage_V.push_back(data_.Nodes[i].SbusVoltage_V[j]);
    }
    for (size_t j=0; j < data_.Nodes[i].Mpu9250.size(); j++) {
      data_.Mpu9250.push_back(data_.Nodes[i].Mpu9250[j]);
    }
    for (size_t j=0; j < data_.Nodes[i].Bme280.size(); j++) {
      data_.Bme280.push_back(data_.Nodes[i].Bme280[j]);
    }
    for (size_t j=0; j < data_.Nodes[i].uBlox.size(); j++) {
      data_.uBlox.push_back(data_.Nodes[i].uBlox[j]);
    }
    for (size_t j=0; j < data_.Nodes[i].Swift.size(); j++) {
      data_.Swift.push_back(data_.Nodes[i].Swift[j]);
    }
    for (size_t j=0; j < data_.Nodes[i].Ams5915.size(); j++) {
      data_.Ams5915.push_back(data_.Nodes[i].Ams5915[j]);
    }
    for (size_t j=0; j < data_.Nodes[i].Sbus.size(); j++) {
      data_.Sbus.push_back(data_.Nodes[i].Sbus[j]);
    }
    for (size_t j=0; j < data_.Nodes[i].Analog.size(); j++) {
      data_.Analog.push_back(data_.Nodes[i].Analog[j]);
    }
  }
  if (ResetI2cBus_) {
    Wire1.resetBus();
  }
  if (ResetBfsBus_) {
    Wire.resetBus();
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
  Buffer->resize(SerializedDataMetadataSize+
    sizeof(data_.Time_us[0])*data_.Time_us.size()+
    sizeof(InternalMpu9250Sensor::Data)*data_.InternalMpu9250.size()+
    sizeof(InternalBme280Sensor::Data)*data_.InternalBme280.size()+
    sizeof(data_.InputVoltage_V[0])*data_.InputVoltage_V.size()+
    sizeof(data_.RegulatedVoltage_V[0])*data_.RegulatedVoltage_V.size()+
    sizeof(data_.PwmVoltage_V[0])*data_.PwmVoltage_V.size()+
    sizeof(data_.SbusVoltage_V[0])*data_.SbusVoltage_V.size()+
    sizeof(Mpu9250Sensor::Data)*data_.Mpu9250.size()+
    sizeof(Bme280Sensor::Data)*data_.Bme280.size()+
    sizeof(uBloxSensor::Data)*data_.uBlox.size()+
    sizeof(SwiftSensor::Data)*data_.Swift.size()+
    sizeof(SbusSensor::Data)*data_.Sbus.size()+
    sizeof(Ams5915Sensor::Data)*data_.Ams5915.size()+
    sizeof(AnalogSensor::Data)*data_.Analog.size());
    // meta data
    uint8_t AcquireInternalData,NumberPwmVoltageSensor,NumberSbusVoltageSensor,NumberMpu9250Sensor,NumberBme280Sensor,NumberuBloxSensor,NumberSwiftSensor,NumberAms5915Sensor,NumberSbusSensor,NumberAnalogSensor;
    AcquireInternalData = 0x00;
    if (AcquireTimeData_) {
      AcquireInternalData |=  0x01;
    }
    if (AcquireInternalMpu9250Data_) {
      AcquireInternalData |=  0x02;
    }
    if (data_.InternalBme280.size() > 0) {
      AcquireInternalData |=  0x04;
    }
    if (AcquireInputVoltageData_) {
      AcquireInternalData |=  0x08;
    }
    if (AcquireRegulatedVoltageData_) {
      AcquireInternalData |=  0x10;
    }
    NumberPwmVoltageSensor = data_.PwmVoltage_V.size();
    NumberSbusVoltageSensor = data_.SbusVoltage_V.size();
    NumberMpu9250Sensor = data_.Mpu9250.size();
    NumberBme280Sensor = data_.Bme280.size();
    NumberuBloxSensor = data_.uBlox.size();
    NumberSwiftSensor = data_.Swift.size();
    NumberAms5915Sensor = data_.Ams5915.size();
    NumberSbusSensor = data_.Sbus.size();
    NumberAnalogSensor = data_.Analog.size();
    // Serial.println(NumberSbusSensor);
    memcpy(Buffer->data()+BufferLocation,&AcquireInternalData,sizeof(AcquireInternalData));
    BufferLocation+=sizeof(AcquireInternalData);
    memcpy(Buffer->data()+BufferLocation,&NumberPwmVoltageSensor,sizeof(NumberPwmVoltageSensor));
    BufferLocation+=sizeof(NumberPwmVoltageSensor);
    memcpy(Buffer->data()+BufferLocation,&NumberSbusVoltageSensor,sizeof(NumberSbusVoltageSensor));
    BufferLocation+=sizeof(NumberSbusVoltageSensor);
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
    // sensor data
    memcpy(Buffer->data()+BufferLocation,data_.Time_us.data(),data_.Time_us.size()*sizeof(data_.Time_us[0]));
    BufferLocation+=data_.Time_us.size()*sizeof(data_.Time_us[0]);
    memcpy(Buffer->data()+BufferLocation,data_.InternalMpu9250.data(),data_.InternalMpu9250.size()*sizeof(InternalMpu9250Sensor::Data));
    BufferLocation+=data_.InternalMpu9250.size()*sizeof(InternalMpu9250Sensor::Data);
    memcpy(Buffer->data()+BufferLocation,data_.InternalBme280.data(),data_.InternalBme280.size()*sizeof(InternalBme280Sensor::Data));
    BufferLocation+=data_.InternalBme280.size()*sizeof(InternalBme280Sensor::Data);
    memcpy(Buffer->data()+BufferLocation,data_.InputVoltage_V.data(),data_.InputVoltage_V.size()*sizeof(data_.InputVoltage_V[0]));
    BufferLocation+=data_.InputVoltage_V.size()*sizeof(data_.InputVoltage_V[0]);
    memcpy(Buffer->data()+BufferLocation,data_.RegulatedVoltage_V.data(),data_.RegulatedVoltage_V.size()*sizeof(data_.RegulatedVoltage_V[0]));
    BufferLocation+=data_.RegulatedVoltage_V.size()*sizeof(data_.RegulatedVoltage_V[0]);
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

  /* free all sensor resources and reset sensor vectors and states */
  void AircraftSensors::End() {
    Serial.print("Freeing sensors...");
    // free up sensor resources
    classes_.InternalMpu9250.End();
    for (size_t i=0; i < classes_.InternalBme280.size(); i++) {
      classes_.InternalBme280[i].End();
    }
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
    for (size_t i=0; i < classes_.Nodes.size(); i++) {
      classes_.Nodes[i].End();
    }
    // clear class and data vectors
    classes_.InternalBme280.clear();
    classes_.Mpu9250.clear();
    classes_.Bme280.clear();
    classes_.uBlox.clear();
    classes_.Swift.clear();
    classes_.Ams5915.clear();
    classes_.Sbus.clear();
    classes_.Analog.clear();
    classes_.Nodes.clear();
    data_.Time_us.clear();
    data_.InternalMpu9250.clear();
    data_.InternalBme280.clear();
    data_.InputVoltage_V.clear();
    data_.RegulatedVoltage_V.clear();
    data_.PwmVoltage_V.clear();
    data_.SbusVoltage_V.clear();
    data_.Mpu9250.clear();
    data_.Bme280.clear();
    data_.uBlox.clear();
    data_.Swift.clear();
    data_.Ams5915.clear();
    data_.Sbus.clear();
    data_.Analog.clear();
    data_.Nodes.clear();
    // reset data acquisition flags
    AcquireTimeData_ = false;
    AcquireInternalMpu9250Data_ = false;
    AcquireInputVoltageData_ = false;
    AcquireRegulatedVoltageData_ = false;
    AcquirePwmVoltageData_ = false;
    AcquireSbusVoltageData_ = false;
    Serial.println("done!");
  }
