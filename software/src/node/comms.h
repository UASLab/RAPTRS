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


#ifndef COMMS_H_
#define COMMS_H_

#include "mission.h"
#include "Vector.h"
#include "i2c_t3.h"
#include "Arduino.h"

class AircraftBfsComms {
  public:
    enum Message {
      ModeCommand,
      Configuration,
      SensorDataSize,
      SensorData,
      EffectorCommand
    };
    AircraftBfsComms(i2c_t3& bus,uint8_t addr,i2c_pins pins,uint32_t rate);
    void Begin();
    void SendSensorDataSize(std::vector<uint8_t> &DataBuffer);
    void SendSensorData(std::vector<uint8_t> &DataBuffer);
    bool ReceiveModeCommand(AircraftMission::Mode *mode);
    bool ReceiveConfigMessage(std::vector<char> *ConfigString);
    bool ReceiveEffectorCommand(std::vector<float> *EffectorCommands);
    void CheckMessages();
    void SendMessage(Message message,std::vector<uint8_t> &Payload);
    bool ReceiveMessage(Message *message, std::vector<uint8_t> *Payload);
    void GetMessage(Message *message, std::vector<uint8_t> *Payload);
    void OnReceive(void (*function)(size_t len));
    void OnRequest(void (*function)(void));
    bool NewReceived();
    void ClearReceived();
  private:
    i2c_t3 *bus_;
    uint8_t addr_;
    i2c_pins pins_;
    uint32_t rate_;
    uint8_t Buffer_[kUartBufferMaxSize];
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t RxByte_;
    uint16_t ParserState_ = 0;
    uint8_t LengthBuffer_[2];
    uint16_t Length_;
    uint8_t Checksum_[2];
    bool MessageReceived_ = false;
    Message ReceivedMessage_;
    std::vector<uint8_t> ReceivedPayload_;
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};

#endif
