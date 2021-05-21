/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "comms.h"
#include "fmu_messages.h"

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

/* builds and sends a BFS message given a message ID and payload */
void AircraftSocComms::SendMessage(uint8_t message, uint8_t index, uint8_t *Payload, int len) {

  bool ackReq = false;
  SerialLink::MsgType type = SerialLink::MsgType::NOACK;

  if ((message == kModeCommand)||(message == kConfigMesg)) {
    ackReq = true;
    type = SerialLink::MsgType::REQACK;
  }

  bus_->beginTransmission(type);
  bus_->write(message);
  bus_->write(index);
  bus_->write(Payload, len);
  bus_->endTransmission(ackReq);
}

/* parses BFS messages returning message ID and payload on success */
bool AircraftSocComms::ReceiveMessage(uint8_t *message, uint8_t *address, std::vector<uint8_t> *Payload) {
  if (bus_->checkReceived()) {
    *message = (uint8_t) bus_->read();
    *address = (uint8_t) bus_->read();
    // Serial.print("received id: "); Serial.print(*message); Serial.print(" addr: "); Serial.println(*address);
    Payload->resize(bus_->available());
    bus_->read(Payload->data(),Payload->size());
    // bus_->sendStatus(true);
    return true;
  } else {
    return false;
  }
}

void AircraftSocComms::SendAck(uint8_t id, uint8_t subid) {
  message::config_ack_t msg;
  msg.ack_id = id;
  msg.ack_subid = subid;
  msg.pack();
  SendMessage(msg.id, 0, msg.payload, msg.len);
  Serial.print("SendAck: "); Serial.println(id);
}

void AircraftSocComms::SendSensorMessages(AircraftSensors *Sensors) {
  size_t mpu9250_counter = 0;
  size_t bme280_counter = 0;
  size_t ublox_counter = 0;
  size_t swift_counter = 0;
  size_t ams5915_counter = 0;
  size_t sbus_counter = 0;
  size_t analog_counter = 0;
  
  if ( Sensors->AcquireInternalMpu9250Data ) {
    message::data_mpu9250_t msg;
    Sensors->classes.InternalMpu9250.UpdateMessage(&msg);
    msg.pack();
    SendMessage(msg.id, 0, msg.payload, msg.len);
  }
  if ( Sensors->AcquireInternalBme280Data ) {
    message::data_bme280_t msg;
    Sensors->classes.InternalBme280.UpdateMessage(&msg);
    msg.pack();
    SendMessage(msg.id, bme280_counter++, msg.payload, msg.len);
  }
  {
    message::data_mpu9250_short_t msg;
    for ( size_t i = 0; i < Sensors->classes.Mpu9250.size(); i++ ) {
      Sensors->classes.Mpu9250[i].UpdateMessage(&msg);
      msg.pack();
      SendMessage(msg.id, mpu9250_counter++, msg.payload, msg.len);
    }
  }
  {
    message::data_bme280_t msg;
    for ( size_t i = 0; i < Sensors->classes.Bme280.size(); i++ ) {
      Sensors->classes.Bme280[i].UpdateMessage(&msg);
      msg.pack();
      SendMessage(msg.id, i, msg.payload, msg.len);
    }
  }
  {
    message::data_ublox_t msg;
    for ( size_t i = 0; i < Sensors->classes.uBlox.size(); i++ ) {
      Sensors->classes.uBlox[i].UpdateMessage(&msg);
      msg.pack();
      SendMessage(msg.id, ublox_counter++, msg.payload, msg.len);
    }
  }
  {
    message::data_swift_t msg;
    for ( size_t i = 0; i < Sensors->classes.Swift.size(); i++ ) {
      Sensors->classes.Swift[i].UpdateMessage(&msg);
      msg.pack();
      SendMessage(msg.id, swift_counter++, msg.payload, msg.len);
    }
  }
  {
    message::data_sbus_t msg;
    for ( size_t i = 0; i < Sensors->classes.Sbus.size(); i++ ) {
      Sensors->classes.Sbus[i].UpdateMessage(&msg);
      msg.pack();
      SendMessage(msg.id, sbus_counter++, msg.payload, msg.len);
    }
  }
  {
    message::data_ams5915_t msg;
    for ( size_t i = 0; i < Sensors->classes.Ams5915.size(); i++ ) {
      Sensors->classes.Ams5915[i].UpdateMessage(&msg);
      msg.pack();
      SendMessage(msg.id, ams5915_counter++, msg.payload, msg.len);
    }
  }
  {
    message::data_analog_t msg;
    for ( size_t i = 0; i < Sensors->classes.Analog.size(); i++ ) {
      Sensors->classes.Analog[i].UpdateMessage(&msg);
      msg.pack();
      SendMessage(msg.id, analog_counter++, msg.payload, msg.len);
    }
  }
  
  // fixme: Unpack the compound sensor message from each node in order
  // and send the proper individual messages to the SOC.
  for (size_t i = 0; i < Sensors->classes.Nodes.size(); i++) {
    std::vector<uint8_t> NodeBuffer;
    Sensors->classes.Nodes[i].GetMessage(&NodeBuffer);
    size_t counter = 0;
    while ( counter <= NodeBuffer.size() - 3 ) {
      uint8_t id = NodeBuffer[counter++];
      uint8_t index = NodeBuffer[counter++];
      uint8_t len = NodeBuffer[counter++];
      if ( counter + len <= NodeBuffer.size() ) {
        if ( id == message::data_mpu9250_short_id ) {
          SendMessage(id, mpu9250_counter++, NodeBuffer.data(), len);
        } else if ( id == message::data_bme280_id ) {
          SendMessage(id, bme280_counter++, NodeBuffer.data(), len);
        } else if ( id == message::data_ublox_id ) {
          SendMessage(id, ublox_counter++, NodeBuffer.data(), len);
        } else if ( id == message::data_swift_id ) {
          SendMessage(id, swift_counter++, NodeBuffer.data(), len);
        } else if ( id == message::data_sbus_id ) {
          SendMessage(id, sbus_counter++, NodeBuffer.data(), len);
        } else if ( id == message::data_ams5915_id ) {
          SendMessage(id, ams5915_counter++, NodeBuffer.data(), len);
        } else if ( id == message::data_analog_id ) {
          SendMessage(id, analog_counter++, NodeBuffer.data(), len);
        }
      }
      counter += len;
    } // while processing compound message
  } // for each node
  
  if ( Sensors->AcquireTimeData ) {
    message::data_time_t msg;
    Sensors->classes.Time.UpdateMessage(&msg);
    msg.pack();
    SendMessage(msg.id, 0, msg.payload, msg.len);
  }
}
