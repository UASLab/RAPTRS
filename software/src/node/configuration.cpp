/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "configuration.h"

/* Load configuration from EEPROM */
void AircraftConfiguration::Load() {
  uint16_t EepromLocation = 0;
  Serial.println("Loading configuration...");
  config_.BfsAddr = EEPROM.read(EepromLocation);
  Serial.print("\tBFS Address: ");
  Serial.println(config_.BfsAddr);
  Serial.println("done!");
}

/* Update configuration structure from messages */
bool AircraftConfiguration::Update(uint8_t id, std::vector<uint8_t> *Payload, AircraftSensors *AircraftSensorsPtr, AircraftEffectors *AircraftEffectorsPtr) {
  if ( AircraftSensorsPtr->UpdateConfig(id, Payload) ) {
    // successfully parsed an aircraft sensor config message
    return true;
  } else if ( AircraftEffectorsPtr->UpdateConfig(id, Payload) ) {
    // successfully parsed an effector config message
    return true;
  } else {
    // not a configuration message
    return false;
  }
}

/* Returns the BFS address */
uint8_t AircraftConfiguration::GetBfsAddr() {
  return config_.BfsAddr;
}
