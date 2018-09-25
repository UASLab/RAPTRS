/*
flightcode.cxx
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

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "configuration.hxx"
#include "fmu.hxx"
#include "sensor-processing.hxx"
#include "mission.hxx"
#include "control.hxx"
#include "excitation.hxx"
#include "effector.hxx"
#include "telemetry.hxx"
#include "datalog.hxx"
#include "netSocket.h"
#include "telnet.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <iostream>
#include <iomanip>
#include <stdint.h>

int main(int argc, char* argv[]) {
  if (argc!=2) {
      std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
      std::cerr << "Configuration file name needed." << std::endl;
      return -1;
  }
  /* displaying software version information */
  std::cout << "Bolder Flight Systems" << std::endl;
  std::cout << "Flight Software Version " << SoftwareVersion << std::endl << std::endl;
  /* declare classes */
  DefinitionTree GlobalData;
  Configuration Config;
  FlightManagementUnit Fmu;
  SensorProcessing SenProc;
  MissionManager Mission;
  ControlLaws Control;
  ExcitationSystem Excitation;
  AircraftEffectors Effectors;
  DatalogClient Datalog;
  TelemetryClient Telemetry;

  /* initialize classes */
  std::cout << "Initializing software modules." << std::endl;
  std::cout << "\tInitializing FMU..." << std::flush;
  Fmu.Begin();
  std::cout << "done!" << std::endl;

  /* configure classes and register with global defs */
  std::cout << "Configuring aircraft." << std::endl;
  rapidjson::Document AircraftConfiguration;
  std::cout << "\tLoading configuration..." << std::flush;
  Config.LoadConfiguration(argv[1], &AircraftConfiguration);
  std::cout << "done!" << std::endl;

  std::cout << "\tConfiguring flight management unit..." << std::endl;
  Fmu.Configure(AircraftConfiguration,&GlobalData);
  std::cout << "\tdone!" << std::endl;
  GlobalData.PrettyPrint("/Sensors/");
  std::cout << std::endl;

  if (AircraftConfiguration.HasMember("Sensor-Processing")) {
    std::cout << "\tConfiguring sensor processing..." << std::flush;
    SenProc.Configure(AircraftConfiguration["Sensor-Processing"],&GlobalData);
    std::cout << "done!" << std::endl;
    GlobalData.PrettyPrint("/Sensor-Processing/");
    std::cout << std::endl;

    if (AircraftConfiguration.HasMember("Control")&&AircraftConfiguration.HasMember("Mission-Manager")&&AircraftConfiguration.HasMember("Effectors")) {
      std::cout << "\tConfiguring mission manager..." << std::flush;
      Mission.Configure(AircraftConfiguration["Mission-Manager"],&GlobalData);
      std::cout << "done!" << std::endl;
      GlobalData.PrettyPrint("/Mission-Manager/");
      std::cout << std::endl;

      std::cout << "\tConfiguring control laws..." << std::flush;
      Control.Configure(AircraftConfiguration["Control"],&GlobalData);
      std::cout << "done!" << std::endl;
      GlobalData.PrettyPrint("/Control/");
      std::cout << std::endl;

      std::cout << "\tConfiguring effectors..." << std::flush;
      Effectors.Configure(AircraftConfiguration["Effectors"],&GlobalData);
      std::cout << "done!" << std::endl;
      GlobalData.PrettyPrint("/Effectors/");
      std::cout << std::endl;

      if (AircraftConfiguration.HasMember("Excitation")) {
        std::cout << "\tConfiguring excitations..." << std::flush;
        Excitation.Configure(AircraftConfiguration["Excitation"],&GlobalData);
        std::cout << "done!" << std::endl;
        GlobalData.PrettyPrint("/Excitation/");
        std::cout << std::endl;
      }
    }
  }

  if (AircraftConfiguration.HasMember("Telemetry")) {
    std::cout << "\tConfiguring telemetry..." << std::flush;
    Telemetry.Configure(AircraftConfiguration["Telemetry"],&GlobalData);
    std::cout << "done!" << std::endl;
  }

  std::cout << "\tConfiguring datalog..." << std::flush;
  Datalog.RegisterGlobalData(GlobalData);
  std::cout << "done!" << std::endl;
  std::cout << "Entering main loop." << std::endl;

  netInit();                    // do this before creating telnet instance
  UGTelnet telnet( 6500, &GlobalData );
  telnet.open();
  std::cout << "Telnet interface opened on port 6500" << std::endl;
  
  GlobalData.PrettyPrint("/");

  /* main loop */
  while(1) {
    if (Fmu.ReceiveSensorData()) {
      if (SenProc.Configured()&&SenProc.Initialized()) {
        // run mission
        Mission.Run();
        // get and set engaged sensor processing
        SenProc.SetEngagedSensorProcessing(Mission.GetEngagedSensorProcessing());
        // run sensor processing
        SenProc.Run();
        // get and set engaged and armed controllers
        Control.SetEngagedController(Mission.GetEngagedController());
        Control.SetArmedController(Mission.GetArmedController());
        // get and set engaged excitation
        Excitation.SetEngagedExcitation(Mission.GetEngagedExcitation());
        if (Mission.GetEngagedController()!="Fmu") {
          // loop through control levels running excitations and control laws
          for (size_t i=0; i < Control.ActiveControlLevels(); i++) {
            // run excitation
            Excitation.RunEngaged(Control.GetActiveLevel(i));
            // run control
            Control.RunEngaged(i);
          }
          // send effector commands to FMU
          Fmu.SendEffectorCommands(Effectors.Run());
        }
        // run armed excitations
        Excitation.RunArmed();
        // run armed control laws
        Control.RunArmed();

float refV_ms = *GlobalData.GetValuePtr<float*>("/Control/refV_ms");
float refH_m = *GlobalData.GetValuePtr<float*>("/Control/refH_m");

float cmdMotor_nd = *GlobalData.GetValuePtr<float*>("/Control/cmdMotor_nd");
float cmdGear_nd = *GlobalData.GetValuePtr<float*>("/Control/cmdGear_nd");
//
std::string CtrlEngaged = Mission.GetEngagedController();

float tempMPU = *GlobalData.GetValuePtr<float*>("/Sensors/Fmu/Mpu9250/Temperature_C");
float vCellMin = *GlobalData.GetValuePtr<float*>("/Sensor-Processing/MinCellVolt_V");

std::cout << CtrlEngaged << "\t" << tempMPU << "\t" << vCellMin << "\t" << cmdGear_nd << "\t" << cmdMotor_nd << "\t" << refV_ms << "\t" << refH_m << std::endl;

      }
      // run telemetry
      Telemetry.Send();
      // run datalog
      Datalog.LogBinaryData();
      telnet.process();
    }
  }
	return 0;
}
