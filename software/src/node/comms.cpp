/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "comms.h"

/* class declaration, hardware serial bus and baudrate */
AircraftBfsComms::AircraftBfsComms(i2c_t3& bus,uint8_t addr,i2c_pins pins,uint32_t rate) {
  bus_ = &bus;
  addr_ =  addr;
  pins_ = pins;
  rate_ = rate;
}

/* begins communication over the BFS bus */
void AircraftBfsComms::Begin() {
  Serial.print("Initializing communication with FMU...");
  bus_->begin(I2C_SLAVE,addr_,pins_,I2C_PULLUP_EXT,rate_);
  Serial.println("done!");
}

/* sends sensor data message */
void AircraftBfsComms::SendSensorData(std::vector<uint8_t> &DataBuffer) {
  SendMessage(SensorData,DataBuffer);
}

/* sends sensor data size message */
void AircraftBfsComms::SendSensorDataSize(std::vector<uint8_t> &DataBuffer) {
  SendMessage(SensorDataSize,DataBuffer);
}

/* returns mode command if a mode command message has been received */
bool AircraftBfsComms::ReceiveModeCommand(AircraftMission::Mode *mode) {
  if (MessageReceived_) {
    if (ReceivedMessage_ == ModeCommand) {
      if (ReceivedPayload_.size() == 1) {
        MessageReceived_ = false;
        *mode = (AircraftMission::Mode)ReceivedPayload_[0];
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* returns configuration string if a configuration message has been received */
bool AircraftBfsComms::ReceiveConfigMessage(std::vector<char> *ConfigString) {
  if (MessageReceived_) {
    if (ReceivedMessage_ == Configuration) {
      MessageReceived_ = false;
      ConfigString->resize(ReceivedPayload_.size());
      memcpy(ConfigString->data(),ReceivedPayload_.data(),ReceivedPayload_.size());
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* returns effector commands if a effector command message has been received */
bool AircraftBfsComms::ReceiveEffectorCommand(std::vector<float> *EffectorCommands) {
  if (MessageReceived_) {
    if (ReceivedMessage_ == EffectorCommand) {
      MessageReceived_ = false;
      EffectorCommands->resize(ReceivedPayload_.size()/sizeof(float));
      memcpy(EffectorCommands->data(),ReceivedPayload_.data(),ReceivedPayload_.size());
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* checks for valid BFS messages received */
void AircraftBfsComms::CheckMessages() {
  MessageReceived_ = ReceiveMessage(&ReceivedMessage_,&ReceivedPayload_);
}

/* builds and sends a BFS message given a message ID and payload */
void AircraftBfsComms::SendMessage(Message message,std::vector<uint8_t> &Payload) {
  if (Payload.size() < (kUartBufferMaxSize-headerLength_-checksumLength_)) {
    // header
    Buffer_[0] = header_[0];
    Buffer_[1] = header_[1];
    // message ID
    Buffer_[2] = (uint8_t)message;
    // payload length
    Buffer_[3] = Payload.size() & 0xff;
    Buffer_[4] = Payload.size() >> 8;
    // payload
    memcpy(Buffer_+headerLength_,Payload.data(),Payload.size());
    // checksum
    CalcChecksum((size_t)(Payload.size()+headerLength_),Buffer_,Checksum_);
    Buffer_[Payload.size()+headerLength_] = Checksum_[0];
    Buffer_[Payload.size()+headerLength_+1] = Checksum_[1];
    // transmit
    bus_->write(Buffer_,Payload.size()+headerLength_+checksumLength_);
  }
}

/* parses BFS messages returning message ID and payload on success */
bool AircraftBfsComms::ReceiveMessage(Message *message, std::vector<uint8_t> *Payload) {
  while(bus_->available()) {
    RxByte_ = bus_->read();
    // header
    if (ParserState_ < 2) {
      if (RxByte_ == header_[ParserState_]) {
        Buffer_[ParserState_] = RxByte_;
        ParserState_++;
      }
    } else if (ParserState_ == 3) {
      LengthBuffer_[0] = RxByte_;
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    } else if (ParserState_ == 4) {
      LengthBuffer_[1] = RxByte_;
      Length_ = ((uint16_t)LengthBuffer_[1] << 8) | LengthBuffer_[0];
      if (Length_ > (kUartBufferMaxSize-headerLength_-checksumLength_)) {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    } else if (ParserState_ < (Length_ + headerLength_)) {
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    } else if (ParserState_ == (Length_ + headerLength_)) {
      CalcChecksum(Length_ + headerLength_,Buffer_,Checksum_);
      if (RxByte_ == Checksum_[0]) {
        ParserState_++;
      } else {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
    // checksum 1
    } else if (ParserState_ == (Length_ + headerLength_ + 1)) {
      if (RxByte_ == Checksum_[1]) {
        // message ID
        *message = (Message) Buffer_[2];
        // payload size
        Payload->resize(Length_);
        // payload
        memcpy(Payload->data(),Buffer_+headerLength_,Length_);
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return true;
      } else {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
    }
  }
  return false;
}

/* computes a two byte checksum */
void AircraftBfsComms::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}

/* returns the last received message */
void AircraftBfsComms::GetMessage(Message *message, std::vector<uint8_t> *Payload) {
  *message = ReceivedMessage_;
  Payload->resize(ReceivedPayload_.size());
  memcpy(Payload->data(), ReceivedPayload_.data(), ReceivedPayload_.size());
}

/* register function with i2c on receive */
void AircraftBfsComms::OnReceive(void (*function)(size_t len)) {
  bus_->onReceive(function);
}

/* register function with i2c on receive */
void AircraftBfsComms::OnRequest(void (*function)(void)) {
  bus_->onRequest(function);
}

bool AircraftBfsComms::NewReceived() {
  return MessageReceived_;
}

void AircraftBfsComms::ClearReceived() {
  MessageReceived_ = false;
}
