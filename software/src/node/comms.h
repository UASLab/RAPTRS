/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

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
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;

    uint8_t TxBuffer_[kUartBufferMaxSize];
    uint8_t TxChecksum_[2];

    uint8_t RxBuffer_[kUartBufferMaxSize];
    uint8_t RxByte_;
    uint16_t RxParserState_ = 0;
    uint8_t LengthRxBuffer_[2];
    uint16_t RxLength_;
    uint8_t RxChecksum_[2];

    bool MessageReceived_ = false;
    Message ReceivedMessage_;
    std::vector<uint8_t> ReceivedPayload_;
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};
