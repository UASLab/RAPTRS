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

/* update MPU9250 sensor configuration */
void Mpu9250Sensor::UpdateConfig(const char *JsonString) {
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
        if (Config.containsKey("MosiPin")) {
          config_.MosiPin = Config["MosiPin"];
        }
        if (Config.containsKey("MisoPin")) {
          config_.MisoPin = Config["MisoPin"];
        }
        if (Config.containsKey("SckPin")) {
          config_.SckPin = Config["SckPin"];
        }
        if (Config.containsKey("Spi")) {
          config_.Spi = Config["Spi"];
        }
      } else {
        if (Config.containsKey("Address")) {
          config_.Addr = Config["Address"];
        } else {
          while(1){
            Serial.println("ERROR: I2C address missing from MPU9250 configuration.");
          }
        }
        if (Config.containsKey("I2c")) {
          config_.I2c = Config["I2c"];
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
      if (Config.containsKey("I2c")) {
        config_.I2c = Config["I2c"];
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

/* update the BME280 sensor configuration */
void Bme280Sensor::UpdateConfig(const char *JsonString) {
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
        if (Config.containsKey("MosiPin")) {
          config_.MosiPin = Config["MosiPin"];
        }
        if (Config.containsKey("MisoPin")) {
          config_.MisoPin = Config["MisoPin"];
        }
        if (Config.containsKey("SckPin")) {
          config_.SckPin = Config["SckPin"];
        }
        if (Config.containsKey("Spi")) {
          config_.Spi = Config["Spi"];
        }
      } else {
        if (Config.containsKey("Address")) {
          config_.Addr = Config["Address"];
        } else {
          while(1){
            Serial.println("ERROR: I2C address missing from BME280 configuration.");
          }
        }
        if (Config.containsKey("I2c")) {
          config_.I2c = Config["I2c"];
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
      if (Config.containsKey("I2c")) {
        config_.I2c = Config["I2c"];
      }
    }
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
void uBloxSensor::UpdateConfig(const char *JsonString) {
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
void Ams5915Sensor::UpdateConfig(const char *JsonString) {
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
    if (Config.containsKey("I2c")) {
      config_.I2c = Config["I2c"];
    }
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
void SwiftSensor::UpdateConfig(const char *JsonString) {
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
    if (Config.containsKey("I2c")) {
      config_.Static.I2c = Config["I2c"];
      config_.Differential.I2c = Config["I2c"];
    }
    StaticAms.SetConfig(config_.Static);
    DiffAms.SetConfig(config_.Differential);
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
void SbusSensor::UpdateConfig(const char *JsonString) {
  DynamicJsonBuffer ConfigBuffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  if (Config.success()) {

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
void AnalogSensor::UpdateConfig(const char *JsonString) {
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

/* updates the sensor configuration */
void AircraftSensors::UpdateConfig(const char *JsonString) {
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
      if (Sensor["Type"] == "PwmVoltage") {
        if (AcquirePwmVoltageData_) {
          while(1){
            Serial.println("ERROR: Pwm voltage already initialized.");
          }
        }
        AcquirePwmVoltageData_ = true;
      }
      if (Sensor["Type"] == "SbusVoltage") {
        if (AcquireSbusVoltageData_) {
          while(1){
            Serial.println("ERROR: Sbus voltage already initialized.");
          }
        }
        AcquireSbusVoltageData_ = true;
      }
      if (Sensor["Type"] == "Mpu9250") {
        Sensor.printTo(buffer.data(),buffer.size());
        Mpu9250Sensor TempSensor;
        TempSensor.UpdateConfig(buffer.data());
        classes_.Mpu9250.push_back(TempSensor);
      }
      if (Sensor["Type"] == "Bme280") {
        Sensor.printTo(buffer.data(),buffer.size());
        Bme280Sensor TempSensor;
        TempSensor.UpdateConfig(buffer.data());
        classes_.Bme280.push_back(TempSensor);
      }
      if (Sensor["Type"] == "uBlox") {
        Sensor.printTo(buffer.data(),buffer.size());
        uBloxSensor TempSensor;
        TempSensor.UpdateConfig(buffer.data());
        classes_.uBlox.push_back(TempSensor);
      }
      if (Sensor["Type"] == "Swift") {
        Sensor.printTo(buffer.data(),buffer.size());
        SwiftSensor TempSensor;
        TempSensor.UpdateConfig(buffer.data());
        classes_.Swift.push_back(TempSensor);
      }
      if (Sensor["Type"] == "Ams5915") {
        Sensor.printTo(buffer.data(),buffer.size());
        Ams5915Sensor TempSensor;
        TempSensor.UpdateConfig(buffer.data());
        classes_.Ams5915.push_back(TempSensor);
      }
      if (Sensor["Type"] == "Sbus") {
        Sensor.printTo(buffer.data(),buffer.size());
        SbusSensor TempSensor;
        classes_.Sbus.push_back(TempSensor);
      }
      if (Sensor["Type"] == "Analog") {
        Sensor.printTo(buffer.data(),buffer.size());
        AnalogSensor TempSensor;
        TempSensor.UpdateConfig(buffer.data());
        classes_.Analog.push_back(TempSensor);
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
    int status;
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
    int status;
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
    int status;
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
    int status;
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
