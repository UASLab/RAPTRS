/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "WProgram.h"
#include "hardware-defs.h"
#include "comms.h"
#include "mission.h"
#include "configuration.h"
#include "sensors.h"
#include "control.h"
#include "effector.h"
#include "utils.h"
#include "definition-tree.h"
#include "FrSky.h"

// global aircraft data tree
DefinitionTree GlobalData;
// class for communicating with SOC
AircraftSocComms SocComms(kSocUart,kSocBaud);
// class for aircraft level configs (i.e. address, FMU orientation in vehicle, etc)
AircraftConfiguration Config;
// class for sensor configuration and data acquisition
AircraftSensors Sensors;
// class for mission management (modes, states, and reference commands)
AircraftMission Mission;
// class for control laws
ControlLaws Control;
// class for effectors
AircraftEffectors Effectors;
// current mode
AircraftMission::Mode MissionMode;
// current state
AircraftMission::State MissionState;
// requested mode
AircraftMission::Mode RequestedMode;
// effector commands
// std::vector<float> EffectorCommands;
// sensor data to soc
std::vector<uint8_t> DataBuffer;

// runs with the FMU integrated IMU data ready interrupt
void ImuInterrupt() {
  // set the IMU data ready flag
  Mission.SetImuDataReady();
}

uint64_t ts;
uint64_t t;

int main()
{
  // serial port for debug messages
  BifrostSetup();
  Serial.begin(kDebugBaud);
  delay(5000);
  Serial.println("Bolder Flight Systems");
  Serial.print("Flight Management Unit Software Version ");
  Serial.println(SoftwareVersion);
  Serial.println();
  // communication with SOC
  SocComms.Begin();
  // load configuration
  Config.Load();
  // attach IMU interrupt
  pinMode(kMpu9250IntPin,INPUT);
  attachInterrupt(kMpu9250IntPin,ImuInterrupt,RISING);
  // set BFS pins to output
  pinMode(kBfsInt1Pin,OUTPUT);
  pinMode(kBfsInt2Pin,OUTPUT);

  message::data_bifrost_t msg_b;
  while (1) {
    BifrostSend(msg_b.airspeed, msg_b.test_id, msg_b.voltage, msg_b.soc_eng, msg_b.cont_sel, msg_b.ext_eng);
    // update the mission mode
    Mission.UpdateMode(&Sensors,&Control,&Effectors,&GlobalData);
    Mission.GetMode(&MissionMode);
    if (MissionMode == AircraftMission::Run) {
      ts = micros_64();

      // update the mission state
      Mission.UpdateState();
      Mission.GetState(&MissionState);
      if (MissionState == AircraftMission::SyncDataCollection) {
        Mission.ClearImuDataReady();
        // read synchronous sensors
        Sensors.ReadSyncSensors();
// Serial.print("\tRead: ");
// Serial.print(float(micros_64() - ts) * 1e-3, 2);
        // buffer for transmitting data
        Sensors.MakeCompoundMessage(&DataBuffer);
// Serial.print("\tDataBuffer: ");
// Serial.print(DataBuffer.size());
        // transmit data to SOC
        SocComms.SendMessage(message::data_compound_id, DataBuffer.data(), DataBuffer.size());
// Serial.print("\tSync: ");
// Serial.print(float(micros_64() - ts) * 1e-3, 2);

      }
      if (MissionState == AircraftMission::AsyncDataCollection) {
        // read the asynchronous sensors
        Sensors.ReadAsyncSensors();
      }
      if (MissionState == AircraftMission::FlightControl) {
        Mission.ClearFlightControlFlag();
        // run control laws
        for (size_t ControlLevelIndex=0; ControlLevelIndex < Control.ActiveControlLevels(); ControlLevelIndex++) {
          Control.Run(ControlLevelIndex);
        }
        // compute effector PWM and SBUS commands from angles
        Effectors.ComputeOutputs(Mission.ThrottleSafed());
// Serial.print("\tControl: ");
// Serial.print(float(micros_64() - ts) * 1e-3, 2);
      }
      if (MissionState == AircraftMission::EffectorOutput) {
        Mission.ClearEffectorOutputFlag();
        // command the effectors to move
        Effectors.CommandEffectors();
// Serial.print("\tEffectors: ");
// Serial.println(float(micros_64() - ts) * 1e-3, 2);
      }
// Serial.print("\tDone: ");
// Serial.print(float(micros_64() - ts) * 1e-3, 2);
    }
    // check for new messages from SOC
    uint8_t id;
    uint8_t address;
    std::vector<uint8_t> Payload;
    if ( SocComms.ReceiveMessage(&id, &address, &Payload) ) {
      if ( id == message::command_mode_id ) {
        // request mode
        message::command_mode_t msg;
        msg.unpack(Payload.data(), Payload.size());
        Serial.print("new mode: "); Serial.println(msg.mode);
        Mission.SetRequestedMode((AircraftMission::Mode)msg.mode);
      } else if ( MissionMode == AircraftMission::Run ) {
        if (id == message::command_effectors_id ) {
           // receive the effector commands
           if (Mission.UseSocEffectorComands()) {
             message::command_effectors_t msg;
             msg.unpack(Payload.data(), Payload.size());
             Effectors.SetCommands(&msg, Mission.ThrottleSafed());
            }
        } else if (id == message::data_bifrost_id ) {
             msg_b.unpack(Payload.data(), Payload.size());
        }
      } else if (MissionMode == AircraftMission::Configuration) {
        if ( Config.Update(id, address, &Payload, &Mission, &Sensors, &Control, &Effectors, &GlobalData) ) {
          Serial.println("Config.Update() returned true, sending ack");
          SocComms.SendAck(id, 0);
        } else {
          Serial.print("Unhandled message while in Configuration mode, id: ");
          Serial.println(id);
        }
      }
    }
  }
}
