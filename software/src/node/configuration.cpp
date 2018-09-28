/*
configuration.cpp
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

/* Update configuration structure from JSON payloads */
void AircraftConfiguration::Update(const char* JsonString,AircraftSensors *AircraftSensorsPtr,AircraftEffectors *AircraftEffectorsPtr) {
  DynamicJsonBuffer ConfigBuffer;
  std::vector<char> buffer;
  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  buffer.resize(ConfigBuffer.size());
  if (Config.success()) {
    if (Config.containsKey("Sensors")) {
      JsonArray &Sensors = Config["Sensors"];
      for (size_t j=0; j < Sensors.size(); j++) {
        JsonObject &Sensor = Sensors[j];
        Sensor.printTo(buffer.data(),buffer.size());
        AircraftSensorsPtr->UpdateConfig(buffer.data());
      }
    }
    if (Config.containsKey("Effectors")) {
      JsonArray &Effectors = Config["Effectors"];
      Effectors.printTo(buffer.data(),buffer.size());
      AircraftEffectorsPtr->UpdateConfig(buffer.data());
    }
  }
}

/* Returns the BFS address */
uint8_t AircraftConfiguration::GetBfsAddr() {
  return config_.BfsAddr;
}
