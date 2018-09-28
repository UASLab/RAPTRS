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

/* update internal MPU9250 sensor configuration */
void InternalMpu9250Sensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    if (Config.containsKey("SRD")) {
      config_.SRD = Config["SRD"];
    }
    if (Config.containsKey("Rotation")) {
      JsonArray &Rotation = Config["Rotation"];
      if (Rotation.size() != 9) {
        while(1){
          Serial.println("ERROR: internal MPU9250 rotation matrix size incorrect.");
        }
      } else {
        config_.Rotation(0,0) = Rotation[0];
        config_.Rotation(0,1) = Rotation[1];
        config_.Rotation(0,2) = Rotation[2];
        config_.Rotation(1,0) = Rotation[3];
        config_.Rotation(1,1) = Rotation[4];
        config_.Rotation(1,2) = Rotation[5];
        config_.Rotation(2,0) = Rotation[6];
        config_.Rotation(2,1) = Rotation[7];
        config_.Rotation(2,2) = Rotation[8];
      }
    }
    if (Config.containsKey("DLPF-Bandwidth")) {
      if (Config["DLPF-Bandwidth"] == "184Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_184HZ;
      } else if (Config["DLPF-Bandwidth"] == "92Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_92HZ;
      } else if (Config["DLPF-Bandwidth"] == "41Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_41HZ;
      } else if (Config["DLPF-Bandwidth"] == "20Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_20HZ;
      } else if (Config["DLPF-Bandwidth"] == "10Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_10HZ;
      } else if (Config["DLPF-Bandwidth"] == "5Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_5HZ;
      } else {
        while(1){
          Serial.println("ERROR: Requested internal MPU9250 DLPF-Bandwidth is not an available option. Available options: 184Hz, 92Hz, 41Hz, 20Hz, 10Hz, 5Hz");
        }
      }
    }
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelX_mss",&data_.Accel_mss(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelY_mss",&data_.Accel_mss(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelZ_mss",&data_.Accel_mss(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroX_rads",&data_.Gyro_rads(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroY_rads",&data_.Gyro_rads(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroZ_rads",&data_.Gyro_rads(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagX_uT",&data_.Mag_uT(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagY_uT",&data_.Mag_uT(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagZ_uT",&data_.Mag_uT(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Temperature_C);
  } else {
    while(1){
      Serial.println("ERROR: Internal Mpu9250 parse fail.");
      Serial.println(JsonString);
    }
  }
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
  data_.Accel_mss = config_.Rotation * config_.Orientation * Accel_Imu_mss;
  data_.Gyro_rads = config_.Rotation * config_.Orientation * Gyro_Imu_rads;
  data_.Mag_uT = config_.Rotation * config_.Orientation * Mag_Imu_uT;
  data_.Temperature_C = Mpu_->getTemperature_C();
  *DataPtr = data_;
}

/* free the internal MPU9250 sensor resources */
void InternalMpu9250Sensor::End() {
  delete Mpu_;
}

/* update MPU9250 sensor configuration */
void Mpu9250Sensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    if (Config.containsKey("UseSpi")) {
      config_.UseSpi = Config["UseSpi"];
      if (config_.UseSpi) {
        if (Config.containsKey("CsPin")) {
          config_.CsPin = Config["CsPin"];
        } else {
          while(1){
            Serial.println("ERROR: CS Pin missing from MPU9250 configuration.");
          }
        }
      } else {
        if (Config.containsKey("Address")) {
          config_.Addr = Config["Address"];
        } else {
          while(1){
            Serial.println("ERROR: I2C address missing from MPU9250 configuration.");
          }
        }
      }
    } else {
      if (Config.containsKey("Address")) {
        config_.Addr = Config["Address"];
      } else {
        while(1){
          Serial.println("ERROR: I2C address missing from MPU9250 configuration.");
        }
      }
    }
    if (Config.containsKey("Rotation")) {
      JsonArray &Rotation = Config["Rotation"];
      if (Rotation.size() != 9) {
        while(1){
          Serial.println("ERROR: MPU9250 rotation matrix size incorrect.");
        }
      } else {
        config_.Rotation(0,0) = Rotation[0];
        config_.Rotation(0,1) = Rotation[1];
        config_.Rotation(0,2) = Rotation[2];
        config_.Rotation(1,0) = Rotation[3];
        config_.Rotation(1,1) = Rotation[4];
        config_.Rotation(1,2) = Rotation[5];
        config_.Rotation(2,0) = Rotation[6];
        config_.Rotation(2,1) = Rotation[7];
        config_.Rotation(2,2) = Rotation[8];
      }
    }
    if (Config.containsKey("DLPF-Bandwidth")) {
      if (Config["DLPF-Bandwidth"] == "184Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_184HZ;
      } else if (Config["DLPF-Bandwidth"] == "92Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_92HZ;
      } else if (Config["DLPF-Bandwidth"] == "41Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_41HZ;
      } else if (Config["DLPF-Bandwidth"] == "20Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_20HZ;
      } else if (Config["DLPF-Bandwidth"] == "10Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_10HZ;
      } else if (Config["DLPF-Bandwidth"] == "5Hz") {
        config_.Bandwidth = MPU9250::DLPF_BANDWIDTH_5HZ;
      } else {
        while(1){
          Serial.println("ERROR: Requested MPU9250 DLPF-Bandwidth is not an available option. Available options: 184Hz, 92Hz, 41Hz, 20Hz, 10Hz, 5Hz");
        }
      }
    }
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Status",(int8_t*)&data_.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelX_mss",&data_.Accel_mss(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelY_mss",&data_.Accel_mss(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelZ_mss",&data_.Accel_mss(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroX_rads",&data_.Gyro_rads(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroY_rads",&data_.Gyro_rads(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroZ_rads",&data_.Gyro_rads(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagX_uT",&data_.Mag_uT(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagY_uT",&data_.Mag_uT(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagZ_uT",&data_.Mag_uT(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Temperature_C);
  } else {
    while(1){
      Serial.println("ERROR: Mpu9250 sensor failed to parse.");
      Serial.println(JsonString);
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
  data_.Accel_mss = config_.Rotation * Accel_Imu_mss;
  data_.Gyro_rads = config_.Rotation * Gyro_Imu_rads;
  data_.Mag_uT = config_.Rotation * Mag_Imu_uT;
  data_.Temperature_C = Mpu_->getTemperature_C();
  data_.ReadStatus = status_;
  *DataPtr = data_;
  return status_;
}

/* free the MPU9250 sensor resources */
void Mpu9250Sensor::End() {
  delete Mpu_;
}

/* update the internal BME280 sensor configuration */
void InternalBme280Sensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Pressure_Pa",&data_.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Temperature_C);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Humidity_RH",&data_.Humidity_RH);
  } else {
    while(1){
      Serial.println("ERROR: Internal Bme280 parse fail.");
      Serial.println(JsonString);
    }
  }
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
void Bme280Sensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    if (Config.containsKey("UseSpi")) {
      config_.UseSpi = Config["UseSpi"];
      if (config_.UseSpi) {
        if (Config.containsKey("CsPin")) {
          config_.CsPin = Config["CsPin"];
        } else {
          while(1){
            Serial.println("ERROR: CS Pin missing from BME280 configuration.");
          }
        }
      } else {
        if (Config.containsKey("Address")) {
          config_.Addr = Config["Address"];
        } else {
          while(1){
            Serial.println("ERROR: I2C address missing from BME280 configuration.");
          }
        }
      }
    } else {
      if (Config.containsKey("Address")) {
        config_.Addr = Config["Address"];
      } else {
        while(1){
          Serial.println("ERROR: I2C address missing from BME280 configuration.");
        }
      }
    }
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Status",(int8_t*)&data_.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Pressure_Pa",&data_.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Temperature_C);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Humidity_RH",&data_.Humidity_RH);
  } else {
    while(1){
      Serial.println("ERROR: Bme280 sensor failed to parse.");
      Serial.println(JsonString);
    }
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
void uBloxSensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    if (Config.containsKey("Uart")&&Config.containsKey("Baud")) {
      config_.Uart = Config["Uart"];
      config_.Baud = Config["Baud"];
    } else {
      while(1){
        Serial.println("ERROR: UART port or baudrate missing from GPS configuration.");
      }
    }
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Fix",(uint8_t*)&data_.Fix);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/NumberSatellites",&data_.NumberSatellites);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/TOW",&data_.TOW);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Year",&data_.Year);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Month",&data_.Month);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Day",&data_.Day);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Hour",&data_.Hour);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Minute",&data_.Min);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Second",&data_.Sec);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Latitude_rad",&data_.LLA(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Longitude_rad",&data_.LLA(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Altitude_m",&data_.LLA(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/NorthVelocity_ms",&data_.NEDVelocity_ms(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/EastVelocity_ms",&data_.NEDVelocity_ms(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/DownVelocity_ms",&data_.NEDVelocity_ms(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/HorizontalAccuracy_m",&data_.Accuracy(0,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/VerticalAccuracy_m",&data_.Accuracy(1,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/VelocityAccuracy_ms",&data_.Accuracy(2,0));
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/pDOP",&data_.pDOP);
  } else {
    while(1){
      Serial.println("ERROR: uBlox sensor failed to parse.");
      Serial.println(JsonString);
    }
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
    data_.LLA(0,0) = uBloxData_.lat*kD2R;
    data_.LLA(1,0) = uBloxData_.lon*kD2R;
    data_.LLA(2,0) = uBloxData_.hMSL;
    data_.NEDVelocity_ms(0,0) = uBloxData_.velN;
    data_.NEDVelocity_ms(1,0) = uBloxData_.velE;
    data_.NEDVelocity_ms(2,0) = uBloxData_.velD;
    data_.Accuracy(0,0) = uBloxData_.hAcc;
    data_.Accuracy(1,0) = uBloxData_.vAcc;
    data_.Accuracy(2,0) = uBloxData_.sAcc;
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
void Ams5915Sensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    if(Config.containsKey("Address")&&Config.containsKey("Transducer")){
      config_.Addr = Config["Address"];
      if (Config["Transducer"] == "AMS5915-0005-D") {
        config_.Transducer = AMS5915::AMS5915_0005_D;
      } else if (Config["Transducer"] == "AMS5915-0010-D") {
        config_.Transducer = AMS5915::AMS5915_0010_D;
      } else if (Config["Transducer"] == "AMS5915-0005-D-B") {
        config_.Transducer = AMS5915::AMS5915_0005_D_B;
      } else if (Config["Transducer"] == "AMS5915-0010-D-B") {
        config_.Transducer = AMS5915::AMS5915_0010_D_B;
      } else if (Config["Transducer"] == "AMS5915-0020-D") {
        config_.Transducer = AMS5915::AMS5915_0020_D;
      } else if (Config["Transducer"] == "AMS5915-0050-D") {
        config_.Transducer = AMS5915::AMS5915_0050_D;
      } else if (Config["Transducer"] == "AMS5915-0100-D") {
        config_.Transducer = AMS5915::AMS5915_0100_D;
      } else if (Config["Transducer"] == "AMS5915-0020-D-B") {
        config_.Transducer = AMS5915::AMS5915_0020_D_B;
      } else if (Config["Transducer"] == "AMS5915-0050-D-B") {
        config_.Transducer = AMS5915::AMS5915_0050_D_B;
      } else if (Config["Transducer"] == "AMS5915-0100-D-B") {
        config_.Transducer = AMS5915::AMS5915_0100_D_B;
      } else if (Config["Transducer"] == "AMS5915-0200-D") {
        config_.Transducer = AMS5915::AMS5915_0200_D;
      } else if (Config["Transducer"] == "AMS5915-0350-D") {
        config_.Transducer = AMS5915::AMS5915_0350_D;
      } else if (Config["Transducer"] == "AMS5915-1000-D") {
        config_.Transducer = AMS5915::AMS5915_1000_D;
      } else if (Config["Transducer"] == "AMS5915-2000-D") {
        config_.Transducer = AMS5915::AMS5915_2000_D;
      } else if (Config["Transducer"] == "AMS5915-4000-D") {
        config_.Transducer = AMS5915::AMS5915_4000_D;
      } else if (Config["Transducer"] == "AMS5915-7000-D") {
        config_.Transducer = AMS5915::AMS5915_7000_D;
      } else if (Config["Transducer"] == "AMS5915-10000-D") {
        config_.Transducer = AMS5915::AMS5915_10000_D;
      } else if (Config["Transducer"] == "AMS5915-0200-D-B") {
        config_.Transducer = AMS5915::AMS5915_0200_D_B;
      } else if (Config["Transducer"] == "AMS5915-0350-D-B") {
        config_.Transducer = AMS5915::AMS5915_0350_D_B;
      } else if (Config["Transducer"] == "AMS5915-1000-D-B") {
        config_.Transducer = AMS5915::AMS5915_1000_D_B;
      } else if (Config["Transducer"] == "AMS5915-1000-A") {
        config_.Transducer = AMS5915::AMS5915_1000_A;
      } else if (Config["Transducer"] == "AMS5915-1200-B") {
        config_.Transducer = AMS5915::AMS5915_1200_B;
      } else {
        while(1){
          Serial.println("ERROR: Requested AMS5915 transducer is not an available option.");
        }
      }
    } else {
      while(1){
        Serial.println("ERROR: I2C address or transducer missing from pressure sensor configuration.");
      }
    }
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Status",(int8_t*)&data_.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Pressure_Pa",&data_.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Temperature_C);
  } else {
    while(1){
      Serial.println("ERROR: Ams5915 sensor failed to parse.");
      Serial.println(JsonString);
    }
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
void SwiftSensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    if(Config.containsKey("Static")&&Config.containsKey("Differential")){
      JsonObject &Static = Config["Static"];
      if (Static.containsKey("Address")) {
        config_.Static.Addr = Static["Address"];
        config_.Static.Transducer = AMS5915::AMS5915_1200_B;
      } else {
        while(1){
          Serial.println("ERROR: Static pressure I2C address missing from pitot configuration.");
        }
      }
      JsonObject &Diff = Config["Differential"];
      if (Diff.containsKey("Address")&&Diff.containsKey("Transducer")) {
        config_.Differential.Addr = Diff["Address"];
        if (Diff["Transducer"] == "AMS5915-0020-D") {
          config_.Differential.Transducer = AMS5915::AMS5915_0020_D;
        } else if (Diff["Transducer"] == "AMS5915-0005-D") {
          config_.Differential.Transducer = AMS5915::AMS5915_0005_D;
        } else if (Diff["Transducer"] == "AMS5915-0010-D") {
          config_.Differential.Transducer = AMS5915::AMS5915_0010_D;
        } else {
          while(1){
            Serial.println("ERROR: Incompatible differential pressure transducer in pitot configuration.");
          }
        }
      } else {
        while(1){
          Serial.println("ERROR: Differential pressure I2C address or transducer missing from pitot configuration.");
        }
      }
    } else {
      while(1){
        Serial.println("ERROR: Static pressure object or differential pressure object missing from pitot configuration.");
      }
    }
    StaticAms.SetConfig(config_.Static);
    DiffAms.SetConfig(config_.Differential);
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Static/Status",(int8_t*)&data_.Static.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Static/Pressure_Pa",&data_.Static.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Static/Temperature_C",&data_.Static.Temperature_C);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Differential/Status",(int8_t*)&data_.Differential.ReadStatus);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Differential/Pressure_Pa",&data_.Differential.Pressure_Pa);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Differential/Temperature_C",&data_.Differential.Temperature_C);
  } else {
    while(1){
      Serial.println("ERROR: Swift sensor failed to parse.");
      Serial.println(JsonString);
    }
  }
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
void SbusSensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/FailSafe",(uint8_t*)&data_.FailSafe);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/LostFrames",&data_.LostFrames);
    for (size_t j=0; j < 16; j++) {
      DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Channels/" + std::to_string(j),&data_.Channels[j]);
    }
  } else {
    while(1){
      Serial.println("ERROR: Sbus sensor failed to parse.");
      Serial.println(JsonString);
    }
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
void AnalogSensor::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {
    if(Config.containsKey("Channel")){
      config_.Channel = Config["Channel"];
    } else {
      while(1){
        Serial.println("ERROR: Analog channel number missing from analog configuration.");
      }
    }
    if (Config.containsKey("Calibration")) {
      JsonArray &Calibration = Config["Calibration"];
      for (size_t i=0; i < Calibration.size(); i++) {
        config_.Calibration.push_back(Calibration[i]);
      }
    }
    std::string Output = (std::string) Config.get<String>("Output").c_str();
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Voltage_V",(uint8_t*)&data_.Voltage_V);
    DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/CalibratedValue",&data_.CalibratedValue);
  } else {
    while(1){
      Serial.println("ERROR: Analog sensor failed to parse.");
      Serial.println(JsonString);
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
void AnalogSensor::GetData(Data *DataPtr) {
  data_.Voltage_V = ((float)analogRead(kAnalogPins[config_.Channel]))*3.3f/(powf(2,kAnalogReadResolution)-1.0f);
  data_.CalibratedValue = PolyVal(config_.Calibration,data_.Voltage_V);
  *DataPtr = data_;
}

/* free resources used by the analog sensors */
void AnalogSensor::End() {}

/* updates the node configuration */
void SensorNodes::UpdateConfig(const char *JsonString,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  std::vector<char> buffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  buffer.resize(ConfigBuffer.size());
  if (Config.success()) {
    if (Config.containsKey("Address")) {
      // get the BFS address
      config_.BfsAddr = Config["Address"];
      // create a new node
      node_ = new Node(kBfsPort,config_.BfsAddr,kBfsRate);
      // start communication with node
      node_->Begin();
      if (Config.containsKey("Sensors")) {
        // send config messages
        node_->SetConfigurationMode();
        JsonArray &Sensors = Config["Sensors"];
        for (size_t i=0; i < Sensors.size(); i++) {
          JsonObject &Sensor = Sensors[i];
          Sensor.printTo(buffer.data(),buffer.size());
          String ConfigString = String("{\"Sensors\":[") + buffer.data() + String("]}");
          node_->Configure(ConfigString);
          if (Sensor.containsKey("Type")) {
            if (Sensor["Type"] == "PwmVoltage") {
              data_.PwmVoltage_V.resize(1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output,&data_.PwmVoltage_V[0]);
            }
            if (Sensor["Type"] == "SbusVoltage") {
              data_.SbusVoltage_V.resize(1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output,&data_.SbusVoltage_V[0]);
            }
            if (Sensor["Type"] == "Mpu9250") {
              data_.Mpu9250.resize(data_.Mpu9250.size()+1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Status",(int8_t*)&data_.Mpu9250.back().ReadStatus);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelX_mss",&data_.Mpu9250.back().Accel_mss(0,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelY_mss",&data_.Mpu9250.back().Accel_mss(1,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/AccelZ_mss",&data_.Mpu9250.back().Accel_mss(2,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroX_rads",&data_.Mpu9250.back().Gyro_rads(0,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroY_rads",&data_.Mpu9250.back().Gyro_rads(1,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/GyroZ_rads",&data_.Mpu9250.back().Gyro_rads(2,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagX_uT",&data_.Mpu9250.back().Mag_uT(0,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagY_uT",&data_.Mpu9250.back().Mag_uT(1,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/MagZ_uT",&data_.Mpu9250.back().Mag_uT(2,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Mpu9250.back().Temperature_C);
            }
            if (Sensor["Type"] == "Bme280") {
              data_.Bme280.resize(data_.Bme280.size()+1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Status",(int8_t*)&data_.Bme280.back().ReadStatus);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Pressure_Pa",&data_.Bme280.back().Pressure_Pa);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Bme280.back().Temperature_C);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Humidity_RH",&data_.Bme280.back().Humidity_RH);
            }
            if (Sensor["Type"] == "uBlox") {
              data_.uBlox.resize(data_.uBlox.size()+1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Fix",(uint8_t*)&data_.uBlox.back().Fix);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/NumberSatellites",&data_.uBlox.back().NumberSatellites);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/TOW",&data_.uBlox.back().TOW);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Year",&data_.uBlox.back().Year);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Month",&data_.uBlox.back().Month);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Day",&data_.uBlox.back().Day);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Hour",&data_.uBlox.back().Hour);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Minute",&data_.uBlox.back().Min);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Second",&data_.uBlox.back().Sec);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Latitude_rad",&data_.uBlox.back().LLA(0,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Longitude_rad",&data_.uBlox.back().LLA(1,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Altitude_m",&data_.uBlox.back().LLA(2,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/NorthVelocity_ms",&data_.uBlox.back().NEDVelocity_ms(0,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/EastVelocity_ms",&data_.uBlox.back().NEDVelocity_ms(1,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/DownVelocity_ms",&data_.uBlox.back().NEDVelocity_ms(2,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/HorizontalAccuracy_m",&data_.uBlox.back().Accuracy(0,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/VerticalAccuracy_m",&data_.uBlox.back().Accuracy(1,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/VelocityAccuracy_ms",&data_.uBlox.back().Accuracy(2,0));
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/pDOP",&data_.uBlox.back().pDOP);
            }
            if (Sensor["Type"] == "Swift") {
              data_.Swift.resize(data_.Swift.size()+1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Static/Status",(int8_t*)&data_.Swift.back().Static.ReadStatus);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Static/Pressure_Pa",&data_.Swift.back().Static.Pressure_Pa);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Static/Temperature_C",&data_.Swift.back().Static.Temperature_C);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Differential/Status",(int8_t*)&data_.Swift.back().Differential.ReadStatus);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Differential/Pressure_Pa",&data_.Swift.back().Differential.Pressure_Pa);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Differential/Temperature_C",&data_.Swift.back().Differential.Temperature_C);
            }
            if (Sensor["Type"] == "Ams5915") {
              data_.Ams5915.resize(data_.Ams5915.size()+1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Status",(int8_t*)&data_.Ams5915.back().ReadStatus);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Pressure_Pa",&data_.Ams5915.back().Pressure_Pa);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Temperature_C",&data_.Ams5915.back().Temperature_C);
            }
            if (Sensor["Type"] == "Sbus") {
              data_.Sbus.resize(data_.Sbus.size()+1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/FailSafe",(uint8_t*)&data_.Sbus.back().FailSafe);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/LostFrames",&data_.Sbus.back().LostFrames);
              for (size_t j=0; j < 16; j++) {
                DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Channels/" + std::to_string(j),&data_.Sbus.back().Channels[j]);
              }
            }
            if (Sensor["Type"] == "Analog") {
              data_.Analog.resize(data_.Analog.size()+1);
              std::string Output = (std::string) Sensor.get<String>("Output").c_str();
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/Voltage_V",(uint8_t*)&data_.Analog.back().Voltage_V);
              DefinitionTreePtr->InitMember(RootPath+"/"+Output+"/CalibratedValue",&data_.Analog.back().CalibratedValue);
            }
          }
        }
      }
    } else {
      while(1){
        Serial.println("ERROR: Sensor Node parse fail.");
        Serial.println(JsonString);
      }
    }
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
void AircraftSensors::UpdateConfig(const char *JsonString,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  std::vector<char> buffer;
  delayMicroseconds(10);
  Serial.println("Updating sensor configuration...");

  Serial.print("Parsing: ");
  Serial.println(JsonString);
  JsonObject &Sensor = ConfigBuffer.parseObject(JsonString);

  Serial.print("Parsed Size: ");
  buffer.resize(ConfigBuffer.size());

  Serial.print("\tParsed Size: ");
  Serial.print(buffer.size());
  Serial.print("\tSuccess: ");
  Serial.println(Sensor.success());

  if (Sensor.success()) {
    if (Sensor.containsKey("Type")) {
      if (Sensor["Type"] == "Time") {
        Serial.print("\tTime: ");
        if (AcquireTimeData_) {
          while(1){
            Serial.println("ERROR: Time already initialized.");
          }
        }
        AcquireTimeData_ = true;
        data_.Time_us.resize(1);
        std::string Output = (std::string) Sensor.get<String>("Output").c_str();
        DefinitionTreePtr->InitMember(RootPath_+"/"+Output,&data_.Time_us[0]);
        Serial.println("done.");
      }
      if (Sensor["Type"] == "InternalMpu9250") {
        if (AcquireInternalMpu9250Data_) {
          while(1){
            Serial.println("ERROR: Internal MPU9250 already initialized.");
          }
        }
        AcquireInternalMpu9250Data_ = true;
        data_.InternalMpu9250.resize(1);
        Sensor.printTo(buffer.data(),buffer.size());
        classes_.InternalMpu9250.UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);
      }
      if (Sensor["Type"] == "InternalBme280") {
        if (classes_.InternalBme280.size() > 0) {
          while(1){
            Serial.println("ERROR: Internal BME280 already initialized.");
          }
        }
        Sensor.printTo(buffer.data(),buffer.size());
        InternalBme280Sensor TempSensor;
        classes_.InternalBme280.push_back(TempSensor);
        data_.InternalBme280.resize(classes_.InternalBme280.size());
        classes_.InternalBme280.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);
      }
      if (Sensor["Type"] == "InputVoltage") {
        if (AcquireInputVoltageData_) {
          while(1){
            Serial.println("ERROR: Input voltage already initialized.");
          }
        }
        AcquireInputVoltageData_ = true;
        data_.InputVoltage_V.resize(1);
        std::string Output = (std::string) Sensor.get<String>("Output").c_str();
        DefinitionTreePtr->InitMember(RootPath_+"/"+Output,&data_.InputVoltage_V[0]);
      }
      if (Sensor["Type"] == "RegulatedVoltage") {
        if (AcquireRegulatedVoltageData_) {
          while(1){
            Serial.println("ERROR: Regulated voltage already initialized.");
          }
        }
        AcquireRegulatedVoltageData_ = true;
        data_.RegulatedVoltage_V.resize(1);
        std::string Output = (std::string) Sensor.get<String>("Output").c_str();
        DefinitionTreePtr->InitMember(RootPath_+"/"+Output,&data_.RegulatedVoltage_V[0]);
      }
      if (Sensor["Type"] == "PwmVoltage") {
        if (AcquirePwmVoltageData_) {
          while(1){
            Serial.println("ERROR: Pwm voltage already initialized.");
          }
        }
        AcquirePwmVoltageData_ = true;
        data_.PwmVoltage_V.resize(1);
        std::string Output = (std::string) Sensor.get<String>("Output").c_str();
        DefinitionTreePtr->InitMember(RootPath_+"/"+Output,&data_.PwmVoltage_V[0]);
      }
      if (Sensor["Type"] == "SbusVoltage") {
        if (AcquireSbusVoltageData_) {
          while(1){
            Serial.println("ERROR: Sbus voltage already initialized.");
          }
        }
        AcquireSbusVoltageData_ = true;
        data_.SbusVoltage_V.resize(1);
        std::string Output = (std::string) Sensor.get<String>("Output").c_str();
        DefinitionTreePtr->InitMember(RootPath_+"/"+Output,&data_.SbusVoltage_V[0]);
      }
      if (Sensor["Type"] == "Mpu9250") {
        Sensor.printTo(buffer.data(),buffer.size());
        Mpu9250Sensor TempSensor;
        classes_.Mpu9250.push_back(TempSensor);
        data_.Mpu9250.resize(classes_.Mpu9250.size());
        classes_.Mpu9250.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);
      }
      if (Sensor["Type"] == "Bme280") {
        Sensor.printTo(buffer.data(),buffer.size());
        Bme280Sensor TempSensor;
        classes_.Bme280.push_back(TempSensor);
        data_.Bme280.resize(classes_.Bme280.size());
        classes_.Bme280.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);

      }
      if (Sensor["Type"] == "uBlox") {
        Sensor.printTo(buffer.data(),buffer.size());
        uBloxSensor TempSensor;
        classes_.uBlox.push_back(TempSensor);
        data_.uBlox.resize(classes_.uBlox.size());
        classes_.uBlox.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);

      }
      if (Sensor["Type"] == "Swift") {
        Sensor.printTo(buffer.data(),buffer.size());
        SwiftSensor TempSensor;
        classes_.Swift.push_back(TempSensor);
        data_.Swift.resize(classes_.Swift.size());
        classes_.Swift.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);
      }
      if (Sensor["Type"] == "Ams5915") {
        Sensor.printTo(buffer.data(),buffer.size());
        Ams5915Sensor TempSensor;
        classes_.Ams5915.push_back(TempSensor);
        data_.Ams5915.resize(classes_.Ams5915.size());
        classes_.Ams5915.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);
      }
      if (Sensor["Type"] == "Sbus") {
        Sensor.printTo(buffer.data(),buffer.size());
        SbusSensor TempSensor;
        classes_.Sbus.push_back(TempSensor);
        data_.Sbus.resize(classes_.Sbus.size());
        classes_.Sbus.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);
      }
      if (Sensor["Type"] == "Analog") {
        Sensor.printTo(buffer.data(),buffer.size());
        AnalogSensor TempSensor;
        classes_.Analog.push_back(TempSensor);
        data_.Analog.resize(classes_.Analog.size());
        classes_.Analog.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);
      }
      if (Sensor["Type"] == "Node") {
        Sensor.printTo(buffer.data(),buffer.size());
        SensorNodes TempSensor;
        classes_.Nodes.push_back(TempSensor);
        data_.Nodes.resize(classes_.Nodes.size());
        classes_.Nodes.back().UpdateConfig(buffer.data(),RootPath_,DefinitionTreePtr);
      }
    }
  } else {
    while(1){
      Serial.println("ERROR: Sensor Configuration failed to parse.");
      Serial.println(JsonString);
    }
  }
  Serial.println("done!");
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
    classes_.Nodes[i].Begin(&data_.Nodes[i]);

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
    int status;
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
    int status;
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
    int status;
    status = classes_.Swift[i].GetData(&data_.Swift[i]);
    if (status < 0) {
      ResetI2cBus_ = true;
    }
  }
  for (size_t i=0; i < classes_.Ams5915.size(); i++) {
    int status;
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
    int status;
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
