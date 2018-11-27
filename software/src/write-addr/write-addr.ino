/*
write-addr.ino
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
