/*
comms.cpp
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

#include "Node.h"

// Fixme: replace all the implicit messaging with actual messages?

/* class declaration, i2c bus, address, and rate */
Node::Node(i2c_t3& bus,uint8_t addr,uint32_t rate) {
  bus_ = &bus;
  addr_ = addr;
  rate_ = rate;
}

/* begins communication over the BFS bus */
void Node::Begin() {
  bus_->begin();
  bus_->setClock(rate_);
}

/* sends a configuration string to the node */
void Node::Configure(String ConfigString) {
  std::vector<uint8_t> Payload;
  Serial.println(ConfigString);
  Payload.resize(ConfigString.length());
  memcpy(Payload.data(),ConfigString.c_str(),Payload.size());
  SendMessage(Configuration,Payload);
  delay(500);
}

/* sends a configuration string to the node */
void Node::Configure(uint8_t id, std::vector<uint8_t> *Payload) {
  SendMessage(id, *Payload);
  delay(500);
}

void Node::SetConfigurationMode() {
  std::vector<uint8_t> Payload;
  Payload.push_back(ConfigurationMode);
  SendMessage(ModeCommand,Payload);
}

void Node::SetRunMode() {
  std::vector<uint8_t> Payload;
  Payload.push_back(RunMode);
  SendMessage(ModeCommand,Payload);
}

/* reads sensor data from the node */
bool Node::ReadSensorData() {
  std::vector<uint8_t> Payload;
  std::vector<uint8_t> Buffer;
  Message message;
  size_t dataSize = 0;
  BuildMessage(SensorDataSize,Payload,&Buffer);
  bus_->beginTransmission(addr_);
  bus_->write(Buffer.data(),Buffer.size());
  bus_->endTransmission(I2C_NOSTOP,I2cHeaderTimeout_us);
  bus_->requestFrom(addr_,2+headerLength_+checksumLength_,I2C_STOP,I2cHeaderTimeout_us);
  if (ReceiveMessage(&message,&Payload)) {
    if (message == SensorDataSize) {
      dataSize = *(uint16_t *)Payload.data();
      // Serial.print("expected node size: "); Serial.println(dataSize);
    } else {
      return false;
    }
  } else {
    return false;
  }
  Payload.clear();
  BuildMessage(SensorData,Payload,&Buffer);
  bus_->beginTransmission(addr_);
  bus_->write(Buffer.data(),Buffer.size());
  bus_->endTransmission(I2C_NOSTOP,I2cHeaderTimeout_us);
  bus_->requestFrom(addr_,dataSize+headerLength_+checksumLength_,I2C_STOP,I2cDataTimeout_us);
  if (ReceiveMessage(&message,&Payload)) {
    if (message == SensorData) {
      size_t PayloadLocation = 0;
      // Serial.print("received node sensor data: "); Serial.println(Payload.size());
      DataBuffer_ = Payload;
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* returns the sensor data buffer from the last ReadSensorData */
void Node::GetSensorDataBuffer(std::vector<uint8_t> *SensorDataBuffer) {
  // fixme: flatten?
  *SensorDataBuffer = DataBuffer_;
}

/* sends effector commands to node */
void Node::SendEffectorCommand(std::vector<float> Commands) {
  std::vector<uint8_t> Payload;
  Payload.resize(Commands.size()*sizeof(float));
  memcpy(Payload.data(),Commands.data(),Payload.size());
  SendMessage(EffectorCommand,Payload);
}

/* builds and sends a BFS message given a message ID and payload */
void Node::SendMessage(uint8_t message, std::vector<uint8_t> &Payload) {
  std::vector<uint8_t> TxBuffer;
  BuildMessage(message,Payload,&TxBuffer);
  // transmit
  bus_->beginTransmission(addr_);
  bus_->write(TxBuffer.data(),TxBuffer.size());
  bus_->endTransmission(I2C_STOP,I2cDataTimeout_us);
}

/* builds a BFS message given a message ID and payload */
void Node::BuildMessage(uint8_t message, std::vector<uint8_t> &Payload, std::vector<uint8_t> *TxBuffer) {
  if (Payload.size() < (kBufferMaxSize-headerLength_-checksumLength_)) {
    TxBuffer->resize(Payload.size()+headerLength_+checksumLength_);
    // header
    Buffer_[0] = header_[0];
    Buffer_[1] = header_[1];
    // message ID
    Buffer_[2] = message;
    // payload length
    Buffer_[3] = Payload.size() & 0xff;
    Buffer_[4] = Payload.size() >> 8;
    // payload
    memcpy(Buffer_+headerLength_,Payload.data(),Payload.size());
    // checksum
    CalcChecksum((size_t)(Payload.size()+headerLength_),Buffer_,Checksum_);
    Buffer_[Payload.size()+headerLength_] = Checksum_[0];
    Buffer_[Payload.size()+headerLength_+1] = Checksum_[1];
    memcpy(TxBuffer->data(),Buffer_,TxBuffer->size());
  }
}

/* parses BFS messages returning message ID and payload on success */
bool Node::ReceiveMessage(Message *message,std::vector<uint8_t> *Payload) {
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
      if (Length_ > (kBufferMaxSize-headerLength_-checksumLength_)) {
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
void Node::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}
