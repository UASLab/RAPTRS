/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "mission.h"
#include "sensors.h"
#include "SerialLink.h"
#include "Vector.h"
#include "i2c_t3.h"
#include "Arduino.h"

class AircraftSocComms {
  public:
    enum Message {
      kModeCommand,
      kConfigMesg,
      kSensorData,
      kEffectorCommand
    };

    AircraftSocComms(HardwareSerial& bus,uint32_t baud);
    void Begin();
    void SendMessage(uint8_t message, uint8_t index, uint8_t *Payload, int len);
    bool ReceiveMessage(uint8_t *message, uint8_t *address, std::vector<uint8_t> *Payload);
    void SendAck(uint8_t id, uint8_t subid);
    void SendSensorMessages(AircraftSensors *Sensors);
  private:
    SerialLink *bus_;
    uint32_t baud_;
};

