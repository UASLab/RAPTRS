/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "effector.h"
#include "control.h"
#include "mission.h"
#include "sensors.h"
#include "hardware-defs.h"
#include "definition-tree.h"
#include "EEPROM.h"
#include "Arduino.h"

class AircraftConfiguration {
  public:
    void Load();
    bool Update(uint8_t id, uint8_t address, std::vector<uint8_t> *Payload, AircraftMission *AircraftMissionPtr,AircraftSensors *AircraftSensorsPtr,ControlLaws *ControlLawsPtr,AircraftEffectors *AircraftEffectorsPtr,DefinitionTree *DefinitionTreePtr);
    uint8_t GetBfsAddr() {
      return bfsAddr_;
    };
  private:
    uint8_t bfsAddr_;
};
