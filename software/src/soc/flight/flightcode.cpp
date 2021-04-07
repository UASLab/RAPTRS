/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor, Chris Regan, Curt Olson
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
#include "route_mgr.h"
#include "rapidjson/document.h"

#include <iostream>
#include <iomanip>
#include <stdint.h>
#include <vector>
#include <cstring>

// STREAM
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>
#include <stdio.h>
#include "moveAveSTI.h"
#include "moveAveSTI_terminate.h"
#include "moveAveSTI_types.h"

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
  RouteMgr Route;

  // STREAM
  struct0_T r;  // Defined in
  double dataOut;

  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  r.a = 1;
  r.b = 2;   //
  r.c = 10;  // Number of points to do moving average over

  // STREAM
  {
  int idx0;
  for (idx0 = 0; idx0 < 25; idx0++)
    {
    dataOut = moveAveSTI(double(idx0), &r);
    }
	printf("index %i || dataOut=%f\n",idx0,dataOut);
  }
  
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
  // deftree.PrettyPrint("/");
  std::cout << std::endl;

  /* configure FMU */
  std::cout << "\tConfiguring flight management unit..." << std::endl;
  Fmu.Configure(AircraftConfiguration);
  std::cout << "\tdone!" << std::endl;
  // deftree.PrettyPrint("/Sensors/");
  std::cout << std::endl;

  if (AircraftConfiguration.HasMember("Route")) {
    std::cout << "\tConfiguring route following..." << std::endl;
    Route.Configure(AircraftConfiguration["Route"]);
    std::cout << "done!" << std::endl;
    // deftree.PrettyPrint("/Route/");
    std::cout << std::endl;
  }

  if (AircraftConfiguration.HasMember("Sensor-Processing")) {
    std::cout << "\tConfiguring sensor processing..." << std::flush;
    SenProc.Configure(AircraftConfiguration["Sensor-Processing"]);
    std::cout << "done!" << std::endl;
    // deftree.PrettyPrint("/Sensor-Processing/");
    std::cout << std::endl;

    if (AircraftConfiguration.HasMember("Control")&&AircraftConfiguration.HasMember("Mission-Manager")&&AircraftConfiguration.HasMember("Effectors")) {
      std::cout << "\tConfiguring effectors..." << std::flush;
      Effectors.Configure(AircraftConfiguration["Effectors"]);
      std::cout << "done!" << std::endl;
      // deftree.PrettyPrint("/Effectors/");
      std::cout << std::endl;

      std::cout << "\tConfiguring control laws..." << std::flush;
      Control.Configure( AircraftConfiguration["Control"] );
      Control.ConfigureEffectors( Effectors.GetKeys() );
      std::cout << "done!" << std::endl;
      // deftree.PrettyPrint("/Control/");
      std::cout << std::endl;

      if (AircraftConfiguration.HasMember("Excitation")) {
        std::cout << "\tConfiguring excitations..." << std::flush;
        Excitation.Configure(AircraftConfiguration["Excitation"]);
        std::cout << "done!" << std::endl;
        // deftree.PrettyPrint("/Excitation/");
        std::cout << std::endl;
      }

      std::cout << "\tConfiguring mission manager..." << std::flush;
      Mission.Configure(AircraftConfiguration["Mission-Manager"]);
      std::cout << "done!" << std::endl;
      // deftree.PrettyPrint("/Mission-Manager/");
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
  ElementPtr profSenProc = deftree.initElement("/Mission/profSenProc", "Test Sensor-Processing System time us", LOG_UINT32, LOG_NONE);
  ElementPtr profControl = deftree.initElement("/Mission/profSenProc", "Test profControl System time us", LOG_UINT32, LOG_NONE);
  ElementPtr profRoute = deftree.initElement("/Mission/profRoute", "Route System time us", LOG_UINT32, LOG_NONE);
  ElementPtr profResponse = deftree.initElement("/Mission/profResponse", "Time from receive sensor data to send back effector commands us", LOG_UINT32, LOG_NONE);
  uint64_t profMainStart_us = 0;
  uint64_t profBaselineStart_us = 0;
  uint64_t profSenProcStart_us = 0;
  uint64_t profRouteStart_us = 0;
  uint64_t profControlStart_us = 0;
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

      if (SenProc.Initialized()) {
        // Run the Baseline Sensor Processing
        profBaselineStart_us = micros();
        SenProc.RunBaseline(Mode::kEngage); // Always run the Baseline SenProc as engaged.

        // Run mission manager
        Mission.Run();

        // Run the Baseline Control Laws
        Control.SetBaseline(Mission.GetBaselineController());
        Control.RunBaseline(Mission.GetBaselineRunMode());
        Control.EffectorBaseline(Mission.GetBaselineRunMode());
        profBaseline->setInt(micros() - profBaselineStart_us);

        // Setup the Test systems
        SenProc.SetTest(Mission.GetTestSensorProcessing());
        Control.SetTest(Mission.GetTestController());
        Excitation.SetExcitation(Mission.GetExcitation());

        // Run Test Sensor-Processing
        profSenProcStart_us = micros(); // Start Test timer
        if (Mission.GetTestRunMode() > 0) { // Armed or Engaged
          SenProc.RunTest(Mission.GetTestRunMode());
        }
        profSenProc->setInt(micros() - profSenProcStart_us);

        // Run Route Manager
        profRouteStart_us = micros(); // Start Test timer
        if (Mission.GetTestRunMode() > 0) { // Armed or Engaged
          Route.Set_RouteSel(Mission.GetTestRoute());
          Route.Run();
        }
        profRoute->setInt(micros() - profRouteStart_us);

        // Run Test Control and Excitation
        profControlStart_us = micros(); // Start Test timer
        if (Mission.GetTestRunMode() > 0) { // Armed or Engaged

          // Loop through control levels running excitations and control laws
          std::vector<std::string> ControlLevels = Control.GetTestLevels();
          for (size_t i=0; i < ControlLevels.size(); i++) {
            // Run excitation at active level
            Excitation.Run(ControlLevels[i]);

            // Run controller at active level
            Control.SetLevel(ControlLevels[i]);
            Control.RunTest(Mission.GetTestRunMode());
          }

          // Copy the /Control/Test Effectors to /Control
          Control.EffectorTest(Mission.GetTestRunMode());
        }
        profControl->setInt(micros() - profControlStart_us);

        profResponse->setInt(micros() - profResponseStart_us);

        // Write out commands for Sim
        if ( sim ) {
          sim_cmd_update();
        }

        // Send effector commands to FMU
        Fmu.SendEffectorCommands(Effectors.Run());



        // Print some status
        float tCurr_ms = 1e-3 * (deftree.getElement("/Sensors/Fmu/Time_us") -> getFloat());
        float dt_ms = tCurr_ms - timePrev_ms;
        timePrev_ms = tCurr_ms;

        std::cout << Mission.GetBaselineController() << ":" << Mission.GetBaselineRunMode() << "\t"
                  << Mission.GetTestController() << ":" << Mission.GetTestRunMode() << "\t"
                  << "Route: " << Mission.GetTestRoute() << "\t"
                  << "Excite: " << Mission.GetExcitation() << "\t"
                  << "dt (ms):  " << dt_ms
                  << std::endl;
      }

      // Run Telemetry
      Telemetry.Send();

      // Run Datalog
      Datalog.LogBinaryData();

      // Send Bifrost data
      Fmu.SendBifrostData();

      // Process Telnet
      telnet.process();

      // Profile
      profMainLoop->setInt(micros() - profMainStart_us);
    }
  }

  return 0;
}
