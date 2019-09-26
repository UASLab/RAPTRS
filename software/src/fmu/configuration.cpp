/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "configuration.h"
#include "fmu_messages.h"

/* Load configuration from EEPROM */
void AircraftConfiguration::Load() {
  uint16_t EepromLocation = 0;
  Serial.println("Loading configuration...");
  config_.BfsAddr = EEPROM.read(EepromLocation);
  Serial.print("\tBFS Address: ");
  Serial.println(config_.BfsAddr);
  Serial.println("done!");
}

/* Update configuration structure from message */
bool AircraftConfiguration::Update(uint8_t id, uint8_t address, std::vector<uint8_t> *Payload, AircraftMission *AircraftMissionPtr,AircraftSensors *AircraftSensorsPtr,ControlLaws *ControlLawsPtr,AircraftEffectors *AircraftEffectorsPtr,DefinitionTree *DefinitionTreePtr) {
  if ( AircraftSensorsPtr->UpdateConfig(id, address, Payload, DefinitionTreePtr) ) {
    return true;
  } else if ( AircraftEffectorsPtr->UpdateConfig(id, address, Payload, DefinitionTreePtr) ) {
    return true;
  } else if ( AircraftMissionPtr->UpdateConfig(id, Payload, DefinitionTreePtr) ) {
    return true;
  } else if ( ControlLawsPtr->Configure(id, Payload, DefinitionTreePtr) ) {
    return true;
  } else {
    Serial.print("Unknown config msg in fmu/configuration.cpp:"); Serial.println(id);
  }
  return false;
}
