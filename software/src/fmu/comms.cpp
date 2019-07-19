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

/* parses BFS messages returning message ID and payload on success */
bool AircraftSocComms::ReceiveOtherMessage(uint8_t *message, uint8_t *address, std::vector<uint8_t> *Payload) {
  if (MessageReceived_) {
    *message = ReceivedMessage_;
    *address = ReceivedAddress_;
    Payload->resize(ReceivedPayload_.size());
    memcpy(Payload->data(),ReceivedPayload_.data(),ReceivedPayload_.size());
    return true;
  }
  return false;
}

/* returns mode command if a mode command message has been received */
bool AircraftSocComms::ReceiveModeCommand(AircraftMission::Mode *mode) {
  if (MessageReceived_) {
    if (ReceivedMessage_ == message::mode_command_id) {
      MessageReceived_ = false;
      cmd_msg.unpack(ReceivedPayload_.data(), ReceivedPayload_.size());
      Serial.print("new mode: "); Serial.println(cmd_msg.mode);
      if (cmd_msg.len == 1) {
        *mode = (AircraftMission::Mode)cmd_msg.mode;
        return true;
      } else {
        return false;
      }
    } else {
      Serial.println("ReceiveModeCommand(): not a mode message");
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
    if (ReceivedMessage_ == message::effector_command_id) {
      MessageReceived_ = false;
      effector_msg.unpack(ReceivedPayload_.data(), ReceivedPayload_.size());
      EffectorCommands->resize(effector_msg.num_active);
      for ( size_t i = 0; i < effector_msg.num_active; i++ ) {
	(*EffectorCommands)[i] = effector_msg.command[i];
      }
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
  MessageReceived_ = ReceiveMessage(&ReceivedMessage_, &ReceivedAddress_, &ReceivedPayload_);
}

/* builds and sends a BFS message given a message ID and payload */
void AircraftSocComms::SendMessage(Message message,std::vector<uint8_t> &Payload) {
  SendMessage(message, Payload.data(), Payload.size());
}

/* builds and sends a BFS message given a message ID and payload */
void AircraftSocComms::SendMessage(uint8_t message, uint8_t *Payload, int len) {
  bus_->beginTransmission();
  bus_->write(message);
  bus_->write(0);
  bus_->write(Payload, len);
  bus_->sendTransmission();
}

/* parses BFS messages returning message ID and payload on success */
bool AircraftSocComms::ReceiveMessage(uint8_t *message, uint8_t *address, std::vector<uint8_t> *Payload) {
  if (bus_->checkReceived()) {
    *message = (uint8_t) bus_->read();
    *address = (uint8_t) bus_->read();
    Serial.print("received id: "); Serial.print(*message); Serial.print(" addr: "); Serial.println(*address);
    Payload->resize(bus_->available());
    bus_->read(Payload->data(),Payload->size());
    bus_->sendStatus(true);
    return true;
  } else {
    return false;
  }
}

void AircraftSocComms::ClearReceived() {
  MessageReceived_ = false;
}

void AircraftSocComms::SendAck(uint8_t id, uint8_t subid) {
  message::config_ack_t msg;
  msg.ack_id = id;
  msg.ack_subid = subid;
  msg.pack();
  SendMessage(msg.id, msg.payload, msg.len);
  Serial.print("SendAck: "); Serial.println(id);
}
