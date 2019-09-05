/*
node.ino
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

#include "WProgram.h"
#include "hardware-defs.h"
#include "comms.h"
#include "mission.h"
#include "configuration.h"
#include "sensors.h"
#include "effector.h"
#include "utils.h"

// class for communicating with FMU
AircraftBfsComms *BfsComms;
// class for aircraft level configs (i.e. address, etc)
AircraftConfiguration Config;
// class for sensor configuration and data acquisition
AircraftSensors Sensors;
// class for mission management (modes and states)
AircraftMission Mission;
// class for effectors
AircraftEffectors Effectors;
// current mode
AircraftMission::Mode MissionMode;
// current state
AircraftMission::State MissionState;
// requested mode
AircraftMission::Mode RequestedMode;
// effector commands
std::vector<float> EffectorCommands;
// buffers for transmitting data messages
std::vector<uint8_t> SizeBuffer;
std::vector<uint8_t> DataBuffer;

unsigned long tnow, tprev;

// send messages on request
void RequestMessage() {
  AircraftBfsComms::Message message;
  std::vector<uint8_t> Payload;
  BfsComms->GetMessage(&message, &Payload);
  if (message == AircraftBfsComms::SensorDataSize) {
    BfsComms->SendSensorDataSize(SizeBuffer);
  }
  if (message == AircraftBfsComms::SensorData) {
    BfsComms->SendSensorData(DataBuffer);
  }
}

// parse messages on receive
void ReceiveMessage(size_t MessageSize) {
  BfsComms->CheckMessages();
}

// sync sensor reading off BFS bus
void SensorInterrupt() {
  // set the sync data collection flag
  Mission.SetSyncDataCollection();
}

// sync effector output off BFS bus
void EffectorInterrupt() {
  // set the effector output flag
  Mission.SetSyncEffectorOutput();
}

void setup() {
  // serial port for debug messages
  Serial.begin(kDebugBaud);
  delay(5000);
  Serial.println("Bolder Flight Systems");
  Serial.print("Node Software Version ");
  Serial.println(SoftwareVersion);
  Serial.println();
  // load configuration
  Config.Load();
  // initialize communication
  BfsComms = new AircraftBfsComms(kBfsPort,Config.GetBfsAddr(),kBfsPins,kBfsRate);
  BfsComms->Begin();
  // setup functions for acting on receive and request
  BfsComms->OnReceive(ReceiveMessage);
  BfsComms->OnRequest(RequestMessage);
  // attach sync data collection interrupt
  pinMode(kSyncDataCollectionIntPin,INPUT);
  attachInterrupt(kSyncDataCollectionIntPin,SensorInterrupt,RISING);
  // attach sync effector output interrupt
  pinMode(kSyncEffectorIntPin,INPUT);
  attachInterrupt(kSyncEffectorIntPin,EffectorInterrupt,RISING);
}

void loop() {
  // update the mission mode
  Mission.UpdateMode(&Sensors,&Effectors);
  Mission.GetMode(&MissionMode);
  if (MissionMode == AircraftMission::Run) {
    // update the mission state
    Mission.UpdateState();
    Mission.GetState(&MissionState);
    if (MissionState == AircraftMission::SyncDataCollection) {
      Mission.ClearSyncDataCollection();
      // read synchronous sensors
      Sensors.ReadSyncSensors();
      // buffer for transmitting data
      Sensors.MakeCompoundMessage(&DataBuffer, &SizeBuffer);
    }
    if (MissionState == AircraftMission::AsyncDataCollection) {
      // read the asynchronous sensors
      Sensors.ReadAsyncSensors();
    }
    if (MissionState == AircraftMission::EffectorOutput) {
      Mission.ClearSyncEffectorOutput();
      // command the effectors to move
      Effectors.CommandEffectors();
    }
    // effector command
    if (BfsComms->ReceiveEffectorCommand(&EffectorCommands)) {
      Effectors.SetCommands(EffectorCommands);
      Effectors.ComputeOutputs();
    }
  }
  if (MissionMode == AircraftMission::Configuration) {
    // buffer for receiving configurations
    if ( BfsComms->NewReceived() ) {
      AircraftBfsComms::Message message;
      std::vector<uint8_t> Payload;
      BfsComms->GetMessage(&message, &Payload);
      // update configuration
      if ( Config.Update(message, &Payload, &Sensors, &Effectors) ) {
        BfsComms->ClearReceived();
      }
    }
  }
  // request mode
  if (BfsComms->ReceiveModeCommand(&RequestedMode)) {
    Mission.SetRequestedMode(RequestedMode);
  }
}

extern "C" int main() {
  setup();
  while (1) {
    loop();
    yield();
  }
}
