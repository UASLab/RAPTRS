/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "sensors.h"

int TimeSensor::ReadSensor() {
  Time_us = micros_64();
}

void TimeSensor::UpdateMessage(message::data_time_t *msg) {
  msg->time_us = Time_us;
}

/* update internal MPU9250 sensor configuration */
bool InternalMpu9250Sensor::UpdateConfig(message::config_mpu9250_t *msg, std::string RootPath,DefinitionTree *DefinitionTreePtr) {
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
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelX_mss",&AccelX_mss);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelY_mss",&AccelY_mss);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelZ_mss",&AccelZ_mss);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroX_rads",&GyroX_rads);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroY_rads",&GyroY_rads);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroZ_rads",&GyroZ_rads);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/MagX_uT",&MagX_uT);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/MagY_uT",&MagY_uT);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/MagZ_uT",&MagZ_uT);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Temperature_C",&Temperature_C);
  return true;
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
int InternalMpu9250Sensor::ReadSensor() {
  Eigen::Matrix<float,3,1>Accel_Imu_mss;
  Eigen::Matrix<float,3,1>Gyro_Imu_rads;
  Eigen::Matrix<float,3,1>Mag_Imu_uT;

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

  Accel_mss_ = config_.Rotation * config_.Orientation * Accel_Imu_mss;
  Gyro_rads_ = config_.Rotation * config_.Orientation * Gyro_Imu_rads;
  Mag_uT_ = config_.Rotation * config_.Orientation * Mag_Imu_uT;

  AccelX_mss = Accel_mss_(0,0);
  AccelY_mss = Accel_mss_(1,0);
  AccelZ_mss = Accel_mss_(2,0);
  GyroX_rads = Gyro_rads_(0,0);
  GyroY_rads = Gyro_rads_(1,0);
  GyroZ_rads = Gyro_rads_(2,0);
  MagX_uT = Mag_uT_(0,0);
  MagY_uT = Mag_uT_(1,0);
  MagZ_uT = Mag_uT_(2,0);
  Temperature_C = Mpu_->getTemperature_C();

  return status_;
}

void InternalMpu9250Sensor::UpdateMessage(message::data_mpu9250_t *msg) {
  msg->AccelX_mss = AccelX_mss;
  msg->AccelY_mss = AccelY_mss;
  msg->AccelZ_mss = AccelZ_mss;
  msg->GyroX_rads = GyroX_rads;
  msg->GyroY_rads = GyroY_rads;
  msg->GyroZ_rads = GyroZ_rads;
  msg->MagX_uT = MagX_uT;
  msg->MagY_uT = MagY_uT;
  msg->MagZ_uT = MagZ_uT;
  msg->Temperature_C = Temperature_C;
}

/* free the internal MPU9250 sensor resources */
void InternalMpu9250Sensor::End() {
  delete Mpu_;
}

/* update MPU9250 sensor configuration */
bool Mpu9250Sensor::UpdateConfig(message::config_mpu9250_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
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
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Status",(int8_t*)&status_);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelX_mss",&Accel_mss_(0,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelY_mss",&Accel_mss_(1,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/AccelZ_mss",&Accel_mss_(2,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroX_rads",&Gyro_rads_(0,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroY_rads",&Gyro_rads_(1,0));
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/GyroZ_rads",&Gyro_rads_(2,0));
  return true;
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

/* update the internal BME280 sensor configuration */
bool InternalBme280Sensor::UpdateConfig(std::string Output,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Pressure_Pa",&Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&Temperature_C);
  DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Humidity_RH",&Humidity_RH);
  return true;
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
int InternalBme280Sensor::ReadSensor() {
  status_ = Bme_->readSensor();
  Pressure_Pa = Bme_->getPressure_Pa();
  Temperature_C = Bme_->getTemperature_C();
  Humidity_RH = Bme_->getHumidity_RH();
  return status_;
}

void InternalBme280Sensor::UpdateMessage(message::data_bme280_t *msg) {
  msg->ReadStatus = status_;
  msg->Pressure_Pa = Pressure_Pa;
  msg->Temperature_C = Temperature_C;
  msg->Humidity_RH = Humidity_RH;
}

/* free resources used by the internal BME280 sensor */
void InternalBme280Sensor::End() {
  delete Bme_;
}

/* update the BME280 sensor configuration */
bool Bme280Sensor::UpdateConfig(message::config_bme280_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  config_.UseSpi = msg->use_spi;
  if (config_.UseSpi) {
    config_.CsPin = msg->cs_pin;
  } else {
    config_.Addr = msg->i2c_addr;
  }
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Status",(int8_t*)&ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Pressure_Pa",&Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Temperature_C",&Temperature_C);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Humidity_RH",&Humidity_RH);
  return true;
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
int Bme280Sensor::ReadSensor() {
  status_ = Bme_->readSensor();
  Pressure_Pa = Bme_->getPressure_Pa();
  Temperature_C = Bme_->getTemperature_C();
  Humidity_RH = Bme_->getHumidity_RH();
  ReadStatus = status_;
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
bool uBloxSensor::UpdateConfig(message::config_ublox_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  if (msg->uart and msg->baud) {
    config_.Uart = msg->uart;
    config_.Baud = msg->baud;
  } else {
    HardFail("ERROR: uBlox port or baudrate missing from GPS configuration.");
  }
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Fix",(uint8_t*)&Fix);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/NumberSatellites",&NumberSatellites);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/TOW",&TOW);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Year",&Year);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Month",&Month);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Day",&Day);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Hour",&Hour);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Minute",&Min);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Second",&Sec);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Latitude_rad",&Longitude_rad);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Longitude_rad",&Latitude_rad);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Altitude_m",&Altitude_m);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/NorthVelocity_ms",&NorthVelocity_ms);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/EastVelocity_ms",&EastVelocity_ms);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/DownVelocity_ms",&DownVelocity_ms);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/HorizontalAccuracy_m",&HorizontalAccuracy_m);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/VerticalAccuracy_m",&VerticalAccuracy_m);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/VelocityAccuracy_ms",&VelocityAccuracy_ms);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/pDOP",&pDOP);
  return true;
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

/* get the private data structure */
int uBloxSensor::ReadSensor() {
  if (ublox_->read(&uBloxData_)) {
    if (uBloxData_.fixType == 3) {
      Fix = true;
    } else {
      Fix = false;
    }
    NumberSatellites = uBloxData_.numSV;
    TOW = uBloxData_.iTOW;
    Year = uBloxData_.utcYear;
    Month = uBloxData_.utcMonth;
    Day = uBloxData_.utcDay;
    Hour = uBloxData_.utcHour;
    Min = uBloxData_.utcMin;
    Sec = uBloxData_.utcSec;
    Latitude_rad = uBloxData_.lat*kD2R;
    Longitude_rad = uBloxData_.lon*kD2R;
    Altitude_m = uBloxData_.hMSL;
    NorthVelocity_ms = uBloxData_.velN;
    EastVelocity_ms = uBloxData_.velE;
    DownVelocity_ms = uBloxData_.velD;
    HorizontalAccuracy_m = uBloxData_.hAcc;
    VerticalAccuracy_m = uBloxData_.vAcc;
    VelocityAccuracy_ms = uBloxData_.sAcc;
    pDOP = uBloxData_.pDOP;
    return true;
  }
  return false;
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
bool Ams5915Sensor::UpdateConfig(message::config_ams5915_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
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
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Status",(int8_t*)&ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Pressure_Pa",&Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Temperature_C",&Temperature_C);
  return true;
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
int Ams5915Sensor::ReadSensor() {
  status_ = ams_->readSensor();
  if ((status_ > 0) && (ams_->getTemperature_C() < 149.0f)) {
    Pressure_Pa = ams_->getPressure_Pa();
    Temperature_C = ams_->getTemperature_C();
  }
  ReadStatus = status_;
  return status_;
}

/* get data from the AMS5915 */
void Ams5915Sensor::UpdateMessage(message::data_ams5915_t *msg) {
  msg->ReadStatus = ReadStatus;
  msg->Pressure_Pa = Pressure_Pa;
  msg->Temperature_C = Temperature_C;
}

/* free resources used by the AMS5915 */
void Ams5915Sensor::End() {
  delete ams_;
}

/* update the Swift sensor configuration */
bool SwiftSensor::UpdateConfig(message::config_swift_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
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
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Static/Status",(int8_t*)&StaticAms.ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Static/Pressure_Pa",&StaticAms.Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Static/Temperature_C",&StaticAms.Temperature_C);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Differential/Status",(int8_t*)&DiffAms.ReadStatus);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Differential/Pressure_Pa",&DiffAms.Pressure_Pa);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Differential/Temperature_C",&DiffAms.Temperature_C);
  return true;
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
  if ((StaticStatus_ < 0)||(DiffStatus_ < 0)) {
    return -1;
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
bool SbusSensor::UpdateConfig(std::string output, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  DefinitionTreePtr->InitMember(RootPath+"/"+output+"/FailSafe",(uint8_t*)&failsafe_);
  DefinitionTreePtr->InitMember(RootPath+"/"+output+"/LostFrames",&lostframes_);
  for (size_t j=0; j < 16; j++) {
    DefinitionTreePtr->InitMember(RootPath+"/"+output+"/Channels/" + std::to_string(j),&channels_[j]);
  }
  return true;
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

/* get the SBUS private data structure */
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
bool AnalogSensor::UpdateConfig(message::config_analog_t *msg, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
  config_.Channel = msg->channel;
  for (int i=0; i < message::max_calibration; i++) {
    if ( !isnanf(msg->calibration[i]) ) {
      config_.Calibration.push_back(msg->calibration[i]);
    }
  }
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/Voltage_V",(uint8_t*)&Voltage_V);
  DefinitionTreePtr->InitMember(RootPath+"/"+msg->output+"/CalibratedValue",&CalibratedValue);
  return true;
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
  Voltage_V = ((float)analogRead(pin))*3.3f/(powf(2,kAnalogReadResolution)-1.0f);
  CalibratedValue = PolyVal(config_.Calibration, Voltage_V);
}

void AnalogSensor::UpdateMessage(message::data_analog_t *msg) {
  msg->calibrated_value = CalibratedValue;
}

/* free resources used by the analog sensors */
void AnalogSensor::End() {}

SensorNodes::SensorNodes(uint8_t address) {
  config_.BfsAddr = address;
}

/* updates the node configuration */
bool SensorNodes::UpdateConfig(uint8_t id, std::vector<uint8_t> *Payload, std::string RootPath, DefinitionTree *DefinitionTreePtr) {
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
#if 0
  // update local structures and definition tree to match
  if ( id == message::config_basic_id ) {
    message::config_basic_t msg;
    msg.unpack(Payload->data(), Payload->size());
    if (msg.sensor == message::sensor_type::pwm_voltage) {
      data_.PwmVoltage_V.resize(1);
      DefinitionTreePtr->InitMember(RootPath + "/" + msg.output, &data_.PwmVoltage_V[0]);
    } else if (msg.sensor == message::sensor_type::sbus_voltage) {
      data_.SbusVoltage_V.resize(1);
      DefinitionTreePtr->InitMember(RootPath + "/" + msg.output, &data_.SbusVoltage_V[0]);
    } else if (msg.sensor == message::sensor_type::sbus) {
      data_.Sbus.resize(data_.Sbus.size()+1);
      DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/FailSafe",(uint8_t*)&data_.Sbus.back().FailSafe);
      DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/LostFrames",&data_.Sbus.back().LostFrames);
      for (size_t j=0; j < 16; j++) {
        DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Channels/" + std::to_string(j),&data_.Sbus.back().Channels[j]);
      }
    }
  } else if ( id == message::config_mpu9250_id ) {
    message::config_mpu9250_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Mpu9250.resize(data_.Mpu9250.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Status",(int8_t*)&data_.Mpu9250.back().ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/AccelX_mss",&data_.Mpu9250.back().AccelX_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/AccelY_mss",&data_.Mpu9250.back().AccelY_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/AccelZ_mss",&data_.Mpu9250.back().AccelZ_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/GyroX_rads",&data_.Mpu9250.back().GyroX_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/GyroY_rads",&data_.Mpu9250.back().GyroY_ct);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/GyroZ_rads",&data_.Mpu9250.back().GyroZ_ct);
  } else if ( id == message::config_bme280_id ) {
    message::config_bme280_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Bme280.resize(data_.Bme280.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Status",(int8_t*)&data_.Bme280.back().ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Pressure_Pa",&data_.Bme280.back().Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Temperature_C",&data_.Bme280.back().Temperature_C);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Humidity_RH",&data_.Bme280.back().Humidity_RH);
  } else if ( id == message::config_ublox_id ) {
    message::config_ublox_t msg;
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
  } else if ( id == message::config_swift_id ) {
    message::config_swift_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Swift.resize(data_.Swift.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Static/Status",(int8_t*)&data_.Swift.back().Static.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Static/Pressure_Pa",&data_.Swift.back().Static.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Static/Temperature_C",&data_.Swift.back().Static.Temperature_C);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Differential/Status",(int8_t*)&data_.Swift.back().Differential.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Differential/Pressure_Pa",&data_.Swift.back().Differential.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Differential/Temperature_C",&data_.Swift.back().Differential.Temperature_C);
  } else if ( id == message::config_ams5915_id ) {
    message::config_ams5915_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Ams5915.resize(data_.Ams5915.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Status",(int8_t*)&data_.Ams5915.back().ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Pressure_Pa",&data_.Ams5915.back().Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/Temperature_C",&data_.Ams5915.back().Temperature_C);
  } else if ( id == message::config_analog_id ) {
    message::config_analog_t msg;
    msg.unpack(Payload->data(), Payload->size());
    data_.Analog.resize(data_.Analog.size()+1);
    DefinitionTreePtr->InitMember(RootPath+"/"+msg.output+"/CalibratedValue",&data_.Analog.back().CalibratedValue);
  } else {
    HardFail("FMU doesn't know how to handle locally a config message for a Node");
  }
#endif
  return true;
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
void SensorNodes::Begin(/*Data *DataPtr*/) {
  node_->SetRunMode();
  while (1) {
    digitalWriteFast(kBfsInt1Pin,HIGH);
    if (node_->ReadSensorData()) {
      break;
    }
    delay(5);
    digitalWriteFast(kBfsInt1Pin,LOW);
  }
}

/* get data from the nodes */
int SensorNodes::ReadSensor() {
  return node_->ReadSensorData();
}

// fixme: this one line method might easily be able to collapse out?
int SensorNodes::GetMessage(std::vector<uint8_t> *NodeMessageBuffer) {
  node_->GetSensorDataBuffer(NodeMessageBuffer);
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
    // this message is for a node
    // only continue if this is a supported sensor config message ...
    if ( id == message::config_basic_id or id == message::config_mpu9250_id or id == message::config_bme280_id or id == message::config_swift_id or id == message::config_ams5915_id or id == message::config_analog_id ) {
      int found = -1;
      for ( size_t i = 0; i < classes_.Nodes.size(); i++ ) {
        if ( classes_.Nodes[i].GetBfsAddr() == address ) {
          found = i;
        }
      }
      if ( found < 0 ) {
        found = classes_.Nodes.size();
        classes_.Nodes.push_back(SensorNodes(address));
        // data_.Nodes.resize(classes_.Nodes.size());
      }
      classes_.Nodes[found].UpdateConfig(id, Payload, RootPath_, DefinitionTreePtr);
      return true;
    } else {
      return false;
    }
  } else if ( id == message::config_basic_id ) {
    message::config_basic_t msg;
    msg.unpack(Payload->data(), Payload->size());
    if ( msg.sensor == message::sensor_type::time ) {
      Serial.println("Time");
      if (AcquireTimeData_) {
	       HardFail("ERROR: Time already initialized.");
      }
      AcquireTimeData_ = true;
      std::string Output = msg.output;
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &classes_.Time.Time_us);
      return true;
    } else if ( msg.sensor == message::sensor_type::input_voltage ) {
      Serial.println("InputVoltage");
      if (AcquireInputVoltageData_) {
	       HardFail("ERROR: Input voltage already initialized.");
      }
      AcquireInputVoltageData_ = true;
      AnalogSensor::Config config;
      config.DirectPin = kInputVoltagePin;
      config.Calibration.clear();
      config.Calibration.push_back(kInputVoltageScale);
      config.Calibration.push_back(0.0);
      classes_.Analog.push_back(AnalogSensor());
      classes_.Analog.back().SetConfig(config);
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &classes_.Analog.back().CalibratedValue);
      return true;
    } else if ( msg.sensor == message::sensor_type::regulated_voltage ) {
      Serial.println("RegulatedVoltage");
      if (AcquireRegulatedVoltageData_) {
	       HardFail("ERROR: Regulated voltage already initialized.");
      }
      AcquireRegulatedVoltageData_ = true;
      AnalogSensor::Config config;
      config.DirectPin = kRegulatedVoltagePin;
      config.Calibration.clear();
      config.Calibration.push_back(kRegulatedVoltageScale);
      config.Calibration.push_back(0.0);
      classes_.Analog.push_back(AnalogSensor());
      classes_.Analog.back().SetConfig(config);
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &classes_.Analog.back().CalibratedValue);
      return true;
    } else if ( msg.sensor == message::sensor_type::pwm_voltage ) {
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
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &classes_.Analog.back().CalibratedValue);
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
      DefinitionTreePtr->InitMember(RootPath_ + "/" + msg.output, &classes_.Analog.back().CalibratedValue);
      return true;
    } else if ( msg.sensor == message::sensor_type::internal_bme280 ) {
      Serial.println("InternalBme280");
      if (AcquireInternalBme280Data_) {
	       HardFail("ERROR: Internal BME280 already initialized.");
      }
      AcquireInternalBme280Data_ = true;
      classes_.InternalBme280.UpdateConfig(msg.output,RootPath_,DefinitionTreePtr);
      return true;
    } else if ( msg.sensor == message::sensor_type::sbus ) {
      Serial.println("Sbus");
      classes_.Sbus.push_back(SbusSensor());
      classes_.Sbus.back().UpdateConfig(msg.output, RootPath_, DefinitionTreePtr);
      return true;
    }
  } else if ( id == message::config_mpu9250_id ) {
    message::config_mpu9250_t msg;
    msg.unpack(Payload->data(), Payload->size());
    if ( msg.internal ) {
      Serial.println("InternalMpu9250");
      if (AcquireInternalMpu9250Data_) {
	       HardFail("ERROR: Internal MPU9250 already initialized.");
      }
      AcquireInternalMpu9250Data_ = true;
      classes_.InternalMpu9250.UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
      return true;
    } else {
      Serial.println("Mpu9250");
      classes_.Mpu9250.push_back(Mpu9250Sensor());
      classes_.Mpu9250.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
      return true;
    }
  } else if ( id  == message::config_ublox_id ) {
    Serial.println("uBlox");
    message::config_ublox_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.uBlox.push_back(uBloxSensor());
    classes_.uBlox.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message::config_swift_id ) {
    Serial.println("Swift");
    message::config_swift_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Swift.push_back(SwiftSensor());
    classes_.Swift.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message::config_bme280_id ) {
    Serial.println("Bme280");
    message::config_bme280_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Bme280.push_back(Bme280Sensor());
    classes_.Bme280.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message::config_ams5915_id ) {
    Serial.println("Ams5915");
    message::config_ams5915_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Ams5915.push_back(Ams5915Sensor());
    classes_.Ams5915.back().UpdateConfig(&msg, RootPath_, DefinitionTreePtr);
    return true;
  } else if ( id == message::config_analog_id ) {
    Serial.println("Analog");
    message::config_analog_t msg;
    msg.unpack(Payload->data(), Payload->size());
    classes_.Analog.push_back(AnalogSensor());
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
  classes_.InternalBme280.Begin();
  Serial.println(AcquireInternalBme280Data_);

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
    classes_.Nodes[i].Begin(/*&data_.Nodes[i]*/);

    Serial.println("\tGetConfig().. ");
    SensorNodes::Config TempConfig;
    classes_.Nodes[i].GetConfig(&TempConfig);

#if 0
    // FIXME: this is useful output, so we need to find a way to
    // confirm and display what Node-level sensors were successfully
    // configured.

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
#endif
  }
  Serial.println("done!");
}

/* read all synchronous sensors and store in data struct */
void AircraftSensors::ReadSyncSensors() {
  ResetI2cBus_ = false;
  ResetBfsBus_ = false;
  digitalWriteFast(kBfsInt1Pin,HIGH);
  digitalWriteFast(kBfsInt2Pin,LOW);
  if (AcquireTimeData_) {
    classes_.Time.ReadSensor();
  }
  if (AcquireInternalMpu9250Data_) {
    classes_.InternalMpu9250.ReadSensor();
  }
  if (AcquireInternalBme280Data_) {
    classes_.InternalBme280.ReadSensor();
  }
  for (size_t i=0; i < classes_.Mpu9250.size(); i++) {
    int8_t status;
    status = classes_.Mpu9250[i].ReadSensor();
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
    status = classes_.Bme280[i].ReadSensor();
    if (status < 0) {
      Bme280Sensor::Config TempConfig;
      classes_.Bme280[i].GetConfig(&TempConfig);
      if (!TempConfig.UseSpi) {
        ResetI2cBus_ = true;
      }
    }
  }
  for (size_t i=0; i < classes_.uBlox.size(); i++) {
    classes_.uBlox[i].ReadSensor();
  }
  for (size_t i=0; i < classes_.Swift.size(); i++) {
    int8_t status;
    status = classes_.Swift[i].ReadSensor();
    if (status < 0) {
      ResetI2cBus_ = true;
    }
  }
  for (size_t i=0; i < classes_.Ams5915.size(); i++) {
    int8_t status;
    status = classes_.Ams5915[i].ReadSensor();
    if (status < 0) {
      ResetI2cBus_ = true;
    }
  }
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].ReadSensor();
  }
  for (size_t i=0; i < classes_.Analog.size(); i++) {
    classes_.Analog[i].ReadSensor();
    // if ( i == 0 ) {
    //   Serial.print("analog: ");
    //   Serial.println(classes_.Analog[i].CalibratedValue);
    // }
  }
  for (size_t i=0; i < classes_.Nodes.size(); i++) {
    int8_t status;
    status = classes_.Nodes[i].ReadSensor();
    if (!status) {
      ResetBfsBus_ = true;
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
    classes_.uBlox[i].ReadSensor();
  }
  for (size_t i=0; i < classes_.Sbus.size(); i++) {
    classes_.Sbus[i].ReadSensor();
  }
}

static void add_msg(std::vector<uint8_t> *Buffer, uint8_t id, uint8_t index, uint8_t len, uint8_t *payload) {
  if ( Buffer->size() + len > Buffer->capacity() ) {
    Buffer->resize(Buffer->size() + len);
    Serial.print("Notice: increasing Buffer size to: ");
    Serial.println(Buffer->capacity());
  }
  Buffer->push_back(id);
  Buffer->push_back(index);
  Buffer->push_back(len);
  Buffer->insert(Buffer->end(), payload, payload + len);
}

static void add_compound_msg(std::vector<uint8_t> *Buffer, size_t len, uint8_t *payload) {
  if ( Buffer->size() + len > Buffer->capacity() ) {
    Buffer->resize(Buffer->size() + len);
    Serial.print("Notice: increasing Buffer size to: ");
    Serial.println(Buffer->capacity());
  }
  Buffer->insert(Buffer->end(), payload, payload + len);
}

/* get data buffer */
void AircraftSensors::MakeCompoundMessage(std::vector<uint8_t> *Buffer) {
  Buffer->clear();
  if ( AcquireTimeData_ ) {
    message::data_time_t msg;
    classes_.Time.UpdateMessage(&msg);
    msg.pack();
    add_msg(Buffer, msg.id, 0, msg.len, msg.payload);
  }
  if ( AcquireInternalMpu9250Data_ ) {
    message::data_mpu9250_t msg;
    classes_.InternalMpu9250.UpdateMessage(&msg);
    msg.pack();
    add_msg(Buffer, msg.id, 0, msg.len, msg.payload);
  }
  if ( AcquireInternalBme280Data_ ) {
    message::data_bme280_t msg;
    classes_.InternalBme280.UpdateMessage(&msg);
    msg.pack();
    add_msg(Buffer, msg.id, 0, msg.len, msg.payload);
  }
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
  for (size_t i=0; i < classes_.Nodes.size(); i++) {
    std::vector<uint8_t> NodeBuffer;
    classes_.Nodes[i].GetMessage(&NodeBuffer);
    add_compound_msg(Buffer, NodeBuffer.size(), NodeBuffer.data());
  }
}

/* free all sensor resources and reset sensor vectors and states */
void AircraftSensors::End() {
  Serial.print("Freeing sensors...");
  // free up sensor resources
  classes_.InternalMpu9250.End();
  classes_.InternalBme280.End();
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
  classes_.Mpu9250.clear();
  classes_.Bme280.clear();
  classes_.uBlox.clear();
  classes_.Swift.clear();
  classes_.Ams5915.clear();
  classes_.Sbus.clear();
  classes_.Analog.clear();
  classes_.Nodes.clear();
  // data_.Time_us.clear();
  // data_.InternalMpu9250.clear();
  // data_.InternalBme280.clear();
  // data_.InputVoltage_V.clear();
  // data_.RegulatedVoltage_V.clear();
  // data_.PwmVoltage_V.clear();
  // data_.SbusVoltage_V.clear();
  // data_.Mpu9250.clear();
  // data_.Bme280.clear();
  // data_.uBlox.clear();
  // data_.Swift.clear();
  // data_.Ams5915.clear();
  // data_.Sbus.clear();
  // data_.Analog.clear();
  // data_.Nodes.clear();

  // reset data acquisition flags
  AcquireTimeData_ = false;
  AcquireInternalMpu9250Data_ = false;
  AcquireInternalBme280Data_ = false;
  AcquireInputVoltageData_ = false;
  AcquireRegulatedVoltageData_ = false;
  AcquirePwmVoltageData_ = false;
  AcquireSbusVoltageData_ = false;
  Serial.println("done!");
}
