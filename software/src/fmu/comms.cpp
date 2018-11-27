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

#include "comms.h"

/* class declaration, hardware serial bus and baudrate */
AircraftSocComms::AircraftSocComms(HardwareSerial& bus,uint32_t baud) {
  bus_ = new SerialLink(bus);
  baud_ =  baud;
}

/* begins communication over the hardware serial bus at the specified baudrate */
void AircraftSocComms::Begin() {
  Serial.print("Initializing communication with SOC...");
  bus_->begin(baud_);
  Serial.println("done!");
}

/* sends sensor data message */
void AircraftSocComms::SendSensorData(std::vector<uint8_t> &DataBuffer) {
  SendMessage(SensorData,DataBuffer);
}

/* returns mode command if a mode command message has been received */
bool AircraftSocComms::ReceiveModeCommand(AircraftMission::Mode *mode) {
  if (MessageReceived_) {
    if (ReceivedMessage_ == ModeCommand) {
      MessageReceived_ = false;
      if (ReceivedPayload_.size() == 1) {
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
bool AircraftSocComms::ReceiveConfigMessage(std::vector<char> *ConfigString) {
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

/* returns effector command if a mode command message has been received */
bool AircraftSocComms::ReceiveEffectorCommand(std::vector<float> *EffectorCommands) {
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
void AircraftSocComms::CheckMessages() {
  MessageReceived_ = ReceiveMessage(&ReceivedMessage_,&ReceivedPayload_);
}

/* builds and sends a BFS message given a message ID and payload */
void AircraftSocComms::SendMessage(Message message,std::vector<uint8_t> &Payload) {
  bus_->beginTransmission();
  bus_->write((uint8_t) message);
  bus_->write(Payload.data(),Payload.size());
  bus_->sendTransmission();
}

/* parses BFS messages returning message ID and payload on success */
bool AircraftSocComms::ReceiveMessage(Message *message,std::vector<uint8_t> *Payload) {
  if (bus_->checkReceived()) {
    *message = (Message) bus_->read();
    Payload->resize(bus_->available());
    bus_->read(Payload->data(),Payload->size());
    bus_->sendStatus(true);
    return true;
  } else {
    return false;
  }
}
