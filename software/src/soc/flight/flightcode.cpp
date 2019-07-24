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

#include "hardware-defs.h"
#include "definition-tree2.h"
#include "configuration.h"
#include "fmu.h"
#include "sensor-processing.h"
#include "mission.h"
#include "control.h"
#include "excitation.h"
#include "effector.h"
#include "telemetry.h"
#include "datalog.h"
#include "netSocket.h"
#include "telnet.h"
#include "sim-interface.h"
#include "circle_mgr.h"
#include "route_mgr.h"
#include "rapidjson/document.h"

#include <iostream>
#include <iomanip>
#include <stdint.h>
#include <vector>
#include <cstring>

// Note: deftree is defined in common/definitiontree2.h and declared
// (and initialized) globally in common/definitiontree2.cpp.  Any
// source file that includes definitiontree2.h may reference and use
// deftree.
float timePrev_ms = 0;

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
  Configuration Config;
  FlightManagementUnit Fmu;
  SensorProcessing SenProc;
  MissionManager Mission;
  ControlSystem Control;
  ExcitationSystem Excitation;
  AircraftEffectors Effectors;
  DatalogClient Datalog;
  TelemetryClient Telemetry;
  CircleMgr circle_mgr;
  RouteMgr route_mgr;

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

  /* initialize simulation */
  std::cout << "Configuring Simulation HIL..." << std::endl;
  bool sim = sim_init(AircraftConfiguration);
  std::cout << "\tdone!" << std::endl;
  deftree.PrettyPrint("/");
  std::cout << std::endl;

  /* configure FMU */
  std::cout << "\tConfiguring flight management unit..." << std::endl;
  Fmu.Configure(AircraftConfiguration);
  std::cout << "\tdone!" << std::endl;
  deftree.PrettyPrint("/Sensors/");
  std::cout << std::endl;

  if (AircraftConfiguration.HasMember("Sensor-Processing")) {
    std::cout << "\tConfiguring sensor processing..." << std::flush;
    SenProc.Configure(AircraftConfiguration["Sensor-Processing"]);
    std::cout << "done!" << std::endl;
    deftree.PrettyPrint("/Sensor-Processing/");
    std::cout << std::endl;

    if (AircraftConfiguration.HasMember("Control")&&AircraftConfiguration.HasMember("Mission-Manager")&&AircraftConfiguration.HasMember("Effectors")) {
      std::cout << "\tConfiguring effectors..." << std::flush;
      Effectors.Configure(AircraftConfiguration["Effectors"]);
      std::cout << "done!" << std::endl;
      deftree.PrettyPrint("/Effectors/");
      std::cout << std::endl;

      std::cout << "\tConfiguring control laws..." << std::flush;
      Control.Configure(AircraftConfiguration["Control"]);
      std::cout << "done!" << std::endl;
      deftree.PrettyPrint("/Control/");
      std::cout << std::endl;

      if (AircraftConfiguration.HasMember("Excitation")) {
        std::cout << "\tConfiguring excitations..." << std::flush;
        Excitation.Configure(AircraftConfiguration["Excitation"]);
        std::cout << "done!" << std::endl;
        deftree.PrettyPrint("/Excitation/");
        std::cout << std::endl;
      }

      std::cout << "\tConfiguring mission manager..." << std::flush;
      Mission.Configure(AircraftConfiguration["Mission-Manager"]);
      std::cout << "done!" << std::endl;
      deftree.PrettyPrint("/Mission-Manager/");
      std::cout << std::endl;
    }
  }

  if (AircraftConfiguration.HasMember("Telemetry")) {
    std::cout << "\tConfiguring telemetry..." << std::flush;
    Telemetry.Configure(AircraftConfiguration["Telemetry"]);
    std::cout << "done!" << std::endl;
  }

  /* profiling */
  ElementPtr profMainLoop = deftree.initElement("/Mission/profMainLoop", "Main loop time us", LOG_UINT32, LOG_NONE);
  ElementPtr profBaseline = deftree.initElement("/Mission/profBaseline", "Baseline Sensor processing time us", LOG_UINT32, LOG_NONE);
  ElementPtr profTest = deftree.initElement("/Mission/profTest", "Test System time us", LOG_UINT32, LOG_NONE);
  ElementPtr profResponse = deftree.initElement("/Mission/profResponse", "Time from receive sensor data to send back effector commands us", LOG_UINT32, LOG_NONE);
  uint64_t profMainStart_us = 0;
  uint64_t profBaselineStart_us = 0;
  uint64_t profTestStart_us = 0;
  uint64_t profResponseStart_us = 0;

  std::cout << "\tConfiguring datalog..." << std::flush;
  Datalog.RegisterGlobalData();
  std::cout << "done!" << std::endl;
  std::cout << "Entering main loop." << std::endl;

  netInit();                    // do this before creating telnet instance
  UGTelnet telnet( 6500 );
  telnet.open();
  std::cout << "Telnet interface opened on port 6500" << std::endl;

  /* main loop */
  while(1) {
    profMainStart_us = profResponseStart_us = micros();
    if (Fmu.ReceiveSensorData()) {
      if ( sim ) {
        sim_sensor_update(); // update sim sensors
      }
      float time = 1e-6 * (deftree.getElement("/Sensors/Fmu/Time_us") -> getFloat());

      if (SenProc.Initialized()) {
        // Run the Baseline Sensor Processing
        profBaselineStart_us = micros();
        SenProc.RunBaseline(Mode::kEngage); // Always run the Baseline SenProc as engaged.

        // Run mission manager
        Mission.Run();

        // Run the Baseline Control Laws
        Control.SetBaseline(Mission.GetBaselineController());
        Control.RunBaseline(Mission.GetBaselineRunMode());
        profBaseline->setInt(micros() - profBaselineStart_us);

        // Setup the Test systems
        SenProc.SetTest(Mission.GetTestSensorProcessing());
        Control.SetTest(Mission.GetTestController());
        Excitation.SetExcitation(Mission.GetExcitation());

        // Run Test systems
        profTestStart_us = micros(); // Start Test timer
        if (Mission.GetTestRunMode() > 0) { // Armed or Engaged

          // Run Test Sensor-Processing
          SenProc.RunTest(Mission.GetTestRunMode());

          // loop through control levels running excitations and control laws
          std::vector<std::string> ControlLevels = Control.GetTestLevels();

          for (size_t i=0; i < ControlLevels.size(); i++) {
            // Run excitation at active level
            Excitation.Run(ControlLevels[i]);

            // Run controller at active level
            Control.SetLevel(ControlLevels[i]);
            Control.RunTest(Mission.GetTestRunMode());
          }
        }

        profTest->setInt(micros() - profTestStart_us);
        profResponse->setInt(micros() - profResponseStart_us);

        // Write out commands for Sim
        if ( sim ) {
          sim_cmd_update();
        }

        // send effector commands to FMU
        Fmu.SendEffectorCommands(Effectors.Run());

        /* Print some status */
        std::string BaseCntrl = Mission.GetBaselineController();
        GenericFunction::Mode BaseMode = Mission.GetBaselineRunMode();
        std::string TestCntrl = Mission.GetTestController();
        GenericFunction::Mode TestMode = Mission.GetTestRunMode();
        std::string ExcitEngaged = Mission.GetExcitation();

        float timeCurr_ms = 1e-3 * (deftree.getElement("/Sensors/Fmu/Time_us") -> getFloat());
        float dt_ms = timeCurr_ms - timePrev_ms;
        timePrev_ms = timeCurr_ms;

        std::cout << BaseCntrl << ":" << BaseMode << "\t"
                  << TestCntrl << ":" << TestMode << "\t"
                  << ExcitEngaged
                  << "\tdt (ms):  " << dt_ms
                  << std::endl;

      }
      // run telemetry
      Telemetry.Send();
      // run datalog
      Datalog.LogBinaryData();
      telnet.process();
      profMainLoop->setInt(micros()-profMainStart_us);
    }
  }

  return 0;
}
