/*
comms.h
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


#ifndef BFSSENSORCOMMS_H_
#define BFSSENSORCOMMS_H_

#include "Vector.h"
#include "Eigen.h"
#include "i2c_t3.h"
#include "Arduino.h"

class Node {
  public:
    Node(i2c_t3& bus,uint8_t addr,uint32_t rate);
    void Begin();
    void Configure(String ConfigString);
    void Configure(uint8_t id, std::vector<uint8_t> *Payload);
    void SetRunMode();
    void SetConfigurationMode();
    bool ReadSensorData();
    void GetSensorDataBuffer(std::vector<uint8_t> *SensorDataBuffer);
    void SendEffectorCommand(std::vector<float> Commands);
  private:
    enum Message {
      ModeCommand,
      Configuration,
      SensorDataSize,
      SensorData,
      EffectorCommand
    };
    enum Mode {
      ConfigurationMode,
      RunMode
    };
    static const size_t kBufferMaxSize = 4096;
    const uint32_t I2cHeaderTimeout_us = 200;
    const uint32_t I2cDataTimeout_us = 2000;

    i2c_t3 *bus_;
    uint8_t addr_;
    uint32_t rate_;

    const uint8_t header_[2] = {0x42,0x46};
    const size_t headerLength_ = 5;
    const size_t checksumLength_ = 2;

    uint8_t TxBuffer_[kBufferMaxSize];
    uint8_t TxChecksum_[2];

    uint8_t RxBuffer_[kBufferMaxSize];
    uint8_t RxByte_;
    uint16_t RxParserState_ = 0;
    uint8_t LengthRxBuffer_[2];
    uint16_t RxLength_ = 0;
    uint8_t RxChecksum_[2];

    // struct SensorData SensorData_;
    std::vector<uint8_t> DataBuffer_;
    void SendMessage(uint8_t message, std::vector<uint8_t> &Payload);
    void BuildMessage(uint8_t message, std::vector<uint8_t> &Payload, std::vector<uint8_t> *TxBuffer);
    bool ReceiveMessage(Message *message,std::vector<uint8_t> *Payload);
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};

#endif
