/*
Copyright (c) 2018 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

// clears EEPROM and sets BFS Address

#include "EEPROM.h"

size_t EEPROM_LENGTH = 4096;
uint8_t BfsAddr;
String Input;

void setup() {
	Serial.begin(115200);
  while(!Serial){}
  Serial.print("Current BFS address set to: ");
  Serial.println(EEPROM.read(0));
  Serial.println("Please enter a new BFS address (1-127).");
  while (Serial.available() == 0){}
  Input = Serial.readString();
  BfsAddr = Input.toInt();
  if ((BfsAddr == 0) || (BfsAddr > 127)) {
    Serial.println("Requested BFS address out of range!\n");
    setup();
  } else {
    Serial.print("BFS address set to: ");
    for (size_t i=0; i<EEPROM_LENGTH; i++) {
      EEPROM.write(i,0);
    }
    EEPROM.write(0,BfsAddr);
    Serial.println(EEPROM.read(0));
  }
}

void loop() {}
