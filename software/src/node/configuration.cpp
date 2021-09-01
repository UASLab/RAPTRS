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
  bfsAddr_ = EEPROM.read(EepromLocation);
  Serial.print("\tBFS Address: ");
  Serial.println(bfsAddr_);
  Serial.println("done!");
}

/* Update configuration structure from messages */
bool AircraftConfiguration::Update(uint8_t id, std::vector<uint8_t> *Payload, AircraftSensors *AircraftSensorsPtr, AircraftEffectors *AircraftEffectorsPtr) {
  if ( AircraftSensorsPtr->UpdateConfig(id, Payload) ) {
    return true;
  } else if ( AircraftEffectorsPtr->UpdateConfig(id, Payload) ) {
    return true;
  } else {
    Serial.print("Unknown config msg in node/configuration.cpp:"); Serial.println(id);
  }
  return false;
}
