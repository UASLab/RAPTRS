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
#include "telnet.hxx"
#include "FGFS.h"
#include "route_mgr.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <iostream>
#include <iomanip>
#include <stdint.h>

using std::cout;
using std::endl;

// Note: deftree is defined in common/definitiontree2.h and declared
// (and initialized) globally in common/definitiontree2.cpp.  Any
// source file that includes definitiontree2.h may reference and use
// deftree.

float timePrev_s = 0;


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
  ControlLaws Control;
  ExcitationSystem Excitation;
  AircraftEffectors Effectors;
  DatalogClient Datalog;
  TelemetryClient Telemetry;
  FGRouteMgr route_mgr;

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

    if (AircraftConfiguration.HasMember("Route")) {
      std::cout << "\tConfiguring route following..." << std::endl;
      route_mgr.init(AircraftConfiguration["Route"]);
    }

    if (AircraftConfiguration.HasMember("Control")&&AircraftConfiguration.HasMember("Mission-Manager")&&AircraftConfiguration.HasMember("Effectors")) {
      std::cout << "\tConfiguring mission manager..." << std::flush;
      Mission.Configure(AircraftConfiguration["Mission-Manager"]);
      std::cout << "done!" << std::endl;
      deftree.PrettyPrint("/Mission-Manager/");
      std::cout << std::endl;

      std::cout << "\tConfiguring control laws..." << std::flush;
      Control.Configure(AircraftConfiguration["Control"]);
      std::cout << "done!" << std::endl;
      deftree.PrettyPrint("/Control/");
      std::cout << std::endl;

      std::cout << "\tConfiguring effectors..." << std::flush;
      Effectors.Configure(AircraftConfiguration["Effectors"]);
      std::cout << "done!" << std::endl;
      deftree.PrettyPrint("/Effectors/");
      std::cout << std::endl;

      if (AircraftConfiguration.HasMember("Excitation")) {
        std::cout << "\tConfiguring excitations..." << std::flush;
        Excitation.Configure(AircraftConfiguration["Excitation"]);
        std::cout << "done!" << std::endl;
        deftree.PrettyPrint("/Excitation/");
        std::cout << std::endl;
      }
    }
  }

  if (AircraftConfiguration.HasMember("Telemetry")) {
    std::cout << "\tConfiguring telemetry..." << std::flush;
    Telemetry.Configure(AircraftConfiguration["Telemetry"]);
    std::cout << "done!" << std::endl;
  }

  /* profiling */
  ElementPtr main_loop_node = deftree.initElement("/Mission/profMainLoop", "Main loop time us", LOG_UINT32, LOG_NONE);
  ElementPtr sensor_proc_node = deftree.initElement("/Mission/profSencorProcssing", "Sensor processing time us", LOG_UINT32, LOG_NONE);
  ElementPtr control_node = deftree.initElement("/Mission/profControl", "Control time us", LOG_UINT32, LOG_NONE);
  ElementPtr response_node = deftree.initElement("/Mission/profResponse", "Time from receive sensor data to send back effector commands us", LOG_UINT32, LOG_NONE);
  uint64_t main_loop_start = 0;
  uint64_t sensor_proc_start = 0;
  uint64_t control_start = 0;
  uint64_t response_start = 0;
  
  std::cout << "\tConfiguring datalog..." << std::flush;
  Datalog.RegisterGlobalData();
  std::cout << "done!" << std::endl;
  std::cout << "Entering main loop." << std::endl;

  netInit();                    // do this before creating telnet instance
  UGTelnet telnet( 6500 );
  telnet.open();
  std::cout << "Telnet interface opened on port 6500" << std::endl;

  bool fgfs = fgfs_init(AircraftConfiguration);

  /* main loop */
  while(1) {
    main_loop_start = response_start = micros();
    if (Fmu.ReceiveSensorData()) {
      if ( fgfs ) {
        // insert flightgear sim data calls
        fgfs_imu_update();
        fgfs_gps_update();
      }
      if (SenProc.Configured()&&SenProc.Initialized()) {
        // run mission
        Mission.Run();
        // get and set engaged sensor processing
        SenProc.SetEngagedSensorProcessing(Mission.GetEngagedSensorProcessing());
        // run sensor processing
        sensor_proc_start = micros();
        SenProc.Run();
        sensor_proc_node->setInt(micros()-sensor_proc_start);
        if ( fgfs ) {
          fgfs_airdata_update(); // overwrite processed air data
        }
        route_mgr.update();
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
            control_start = micros();
            Control.RunEngaged(i);
            control_node->setInt(micros()-control_start);
          }
          // send effector commands to FMU
          Fmu.SendEffectorCommands(Effectors.Run());
        }
        response_node->setInt(micros()-response_start);
        if ( fgfs ) {
          fgfs_act_update();
        }
        // run armed excitations
        Excitation.RunArmed();
        // run armed control laws
        Control.RunArmed();

        // Print some status
        std::string CtrlEngaged = Mission.GetEngagedController();
        std::string ExcitEngaged = Mission.GetEngagedExcitation();

        float timeCurr_s = 1e-6 * (deftree.getElement("/Sensors/Fmu/Time_us") -> getFloat());
        float dt = timeCurr_s - timePrev_s;
        timePrev_s = timeCurr_s;

        std::cout << CtrlEngaged << "\t" << ExcitEngaged
                  << "\tdt:  " << dt
                  << std::endl;

      }
      // run telemetry
      Telemetry.Send();
      // run datalog
      Datalog.LogBinaryData();
      telnet.process();
      main_loop_node->setInt(micros()-main_loop_start);
    }
  }

  return 0;
}
