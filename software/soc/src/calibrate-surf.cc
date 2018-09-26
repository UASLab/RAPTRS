/*

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
#include "datalog.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "inclinometer.hxx"

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <stdint.h>


#define MaxCmdDim 11

typedef Eigen::Matrix<float, -1, 1, 0, MaxCmdDim, 1> VecCmd;

int main(int argc, char* argv[]) {
  if (argc!=2) {
    std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
    std::cerr << "Configuration file name needed." << std::endl;
    return -1;
  }
  /* displaying software version information */
  std::cout << "Bolder Flight Systems" << std::endl;
  std::cout << "Surface Calibration Version " << SoftwareVersion << std::endl << std::endl;

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

  /* Setup Inclinometer */
  Incline incline;
  InclineData incData;

  // while(1) {
  //   // Test inclinometer
  //   incline.GetAngle(&incData);
  //   std::cout << incData.Angle_deg << std::endl;
  // }

  incline.SetDamping();

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
  if (AircraftConfiguration.HasMember("Sensor-Processing")) {
    std::cout << "\tConfiguring sensor processing..." << std::flush;
    SenProc.Configure(AircraftConfiguration["Sensor-Processing"],&GlobalData);
    std::cout << "done!" << std::endl;
    if (AircraftConfiguration.HasMember("Control")&&AircraftConfiguration.HasMember("Mission-Manager")&&AircraftConfiguration.HasMember("Effectors")) {
      std::cout << "\tConfiguring mission manager..." << std::flush;
      Mission.Configure(AircraftConfiguration["Mission-Manager"],&GlobalData);
      std::cout << "done!" << std::endl;
      std::cout << "\tConfiguring control laws..." << std::flush;
      Control.Configure(AircraftConfiguration["Control"],&GlobalData);
      std::cout << "done!" << std::endl;
      std::cout << "\tConfiguring effectors..." << std::flush;
      Effectors.Configure(AircraftConfiguration["Effectors"],&GlobalData);
      std::cout << "done!" << std::endl;
      if (AircraftConfiguration.HasMember("Excitation")) {
        std::cout << "\tConfiguring excitations..." << std::flush;
        Excitation.Configure(AircraftConfiguration["Excitation"],&GlobalData);
        std::cout << "done!" << std::endl;
      }
    }
  }
  std::cout << "\tConfiguring datalog..." << std::flush;
  Datalog.RegisterGlobalData(GlobalData);
  std::cout << "done!" << std::endl;

  // Define Delays
  int DelayMove = 2000000; // delay for 2 second after servo move
  int DelayRead = 100000; // delay for 100 ms between reads

  int NumRead = 30; // Number of iteration to sample

  int ServoIndx = -1;
  std::string AnalogString;

  // Servo Command Definition
  float CmdStart;
  float CmdEnd;
  int NumCmds = MaxCmdDim;

  VecCmd CmdList_;
  CmdList_.setZero(NumCmds);

  std::vector<float> Commands;
  Commands = Effectors.Run(); // Use Effector class to get the size
  int numEff = Commands.size();

  while(1) {
    for (int i=0; i < numEff; i++) {
      Commands[i] = 0.0;
    }

    std::cout << "The control surface calibration routine requires the Inclinometer connected to the BBB_UART1 " << std::endl;
    std::cout << "The commands sent to the control surface are based on the type of servo." << std::endl;
    std::cout << "Servos are referenced by the order in which they appear in the JSON Config file." << std::endl;
    std::cout << "Potentiometers are referenced by the full path in the defTree (ie. 'Surf/posLTE1' for '/Sensors/Surf/posLTE1/Voltage_V')." << std::endl;
    std::cout << "Effector Pwm calibration should be set to [500, 1500]." << std::endl;
    std::cout << "Effector Sbus calibration should be set to [682, 1024]." << std::endl;
    std::cout << "PWM range is typically 1000 to 2000, SBUS range is 342 to 1706." << std::endl;
    std::cout << "Calibration range should be -0.4 to 0.4 (or there about)." << std::endl;
    std::cout << "Best results are obtained by tranversing nearly the entire surface deflection range; starting with the surface down and incrementally increasing." << std::endl;

    //Get the SBUS or PWM channel number
    std::cout << "Enter the Servo Index Number: ";
    std::cin >> ServoIndx;
    std::cin.ignore(5, '\n');

    // Surface Commands to free-float
    Commands[ServoIndx] = 0.0;
    Fmu.SendEffectorCommands(Commands); // write commands

    //Get the Pot channel number
    std::cout << "Enter the Analog String for Surface Pot (Enter to skip): ";
    std::getline(std::cin, AnalogString);
    std::cin.ignore(50, '\n');
    std::string AnalogPath = "/Sensors/" + AnalogString + "/Voltage_V";
    std::cout << "Reading Analog from: " << AnalogPath << std::endl;

    //Get Starting Command
    std::cout << "Enter the Starting Command: ";
    std::cin >> CmdStart;
    std::cin.ignore(5, '\n');

    //Get Ending Command
    std::cout << "Enter the Ending Command: ";
    std::cin >> CmdEnd;
    std::cin.ignore(5, '\n');

    // Generate the list of servo commands
    CmdList_.setLinSpaced(NumCmds, CmdStart, CmdEnd);

    // Begin the test
    std::cout << "Beginning Test, Surface: " << ServoIndx << std::endl;

    std::cout << "Turn OFF Servo Power, Hold Surface at Zero Position, Press Enter to continue . . . " << std::endl;;
    std::cin.ignore();
    std::cout << "Proceeding " << std::endl;

    usleep(DelayRead); // delay

    // Establish Zero Angle
    float IncAngleSum_deg = 0.0;
    for (int iRead = 0; iRead < NumRead; ++iRead) {

      // Delay
      usleep (DelayRead);

      // Read inclinometer
      incline.GetAngle(&incData);

      // Accumulate Values
      IncAngleSum_deg += incData.Angle_deg;
    }

    // Average values, define as zero angle
    float incAngleZero_deg = IncAngleSum_deg / (float) NumRead;

    std::cout << "Zero Angle: " << incAngleZero_deg << std::endl;

    std::cout << "Beginning Automated Test..." << std::endl;
    std::cout << "Turn on Servo Power, Press Enter to continue . . . " << std::endl;
    std::cin.ignore();
    std::cout << "Proceeding " << std::endl;

    usleep(DelayMove); // delay

    std::cout << "Starting... " << std::endl;

    // Loop Command Steps
    VecCmd incAngleMeas_deg;
    incAngleMeas_deg.setZero(NumCmds);

    VecCmd potValMeas_V;
    potValMeas_V.setZero(NumCmds);

    for (int iCmd = 0; iCmd < NumCmds; ++iCmd) {
      // Command Servo to command
      float cmdCurr = CmdList_[iCmd];

      std::cout << "Servo Command: " << cmdCurr << std::endl;

      // Send the Servo Command
      Commands[ServoIndx] = cmdCurr;
      Fmu.SendEffectorCommands(Commands);

      usleep(DelayMove); // delay

      IncAngleSum_deg = 0.0;
      float* potValTemp_V;
      float potValSum_V;

      Fmu.ReceiveSensorData(); // Attempt to flush the old pot data

      // Read the Inclinometer and pot data
      potValSum_V = 0.0;
      for (int iRead = 0; iRead < NumRead; ++iRead) {
        while (!Fmu.ReceiveSensorData()) // Wait for new FMU data

        // Keep Sending the same servo command
        Fmu.SendEffectorCommands(Commands);

        // Delay
        usleep (DelayRead);

        // Read inclinometer
        incline.GetAngle(&incData);

        // Accumulate Values
        IncAngleSum_deg += incData.Angle_deg;

        // Read Pot fmu
        // if(AnalogString != ""){
          potValTemp_V = GlobalData.GetValuePtr<float*>("/Sensors/" + AnalogString + "/Voltage_V"); // /Sensors/Surf/posLTE1/Voltage_V

          potValSum_V += *potValTemp_V;
        // }
      }

      // Average values
      incAngleMeas_deg[iCmd] = (IncAngleSum_deg / (float) NumRead) - incAngleZero_deg; // Average minus the 'Zero'
      // if(AnalogString != ""){
        potValMeas_V[iCmd] = potValSum_V / (float) NumRead;
      // }

    }

    // Surface Commands to free-float
    Commands[ServoIndx] = 0.0;
    Fmu.SendEffectorCommands(Commands);

    // Print the results
    std::cout << "Calibration Complete" << std::endl;
    std::cout << "%% Surface: " << std::endl;
    std::cout << "% Servo: " << ServoIndx << std::endl;
    std::cout << "CmdList_nd = [" << CmdList_.transpose() << "];" << std::endl;
    std::cout << "CmdList_us = polyval(cmdCalDefault, CmdList_nd);" << std::endl;
    std::cout << "incAngleMeas_rad = [" << incAngleMeas_deg.transpose() << "] * d2r;" << std::endl;
    std::cout << "fitOrderServo = 1; % Start with Linear, alter as required" << std::endl;
    std::cout << "[polyCoefServo, cmdZero] = ServoCal(CmdList_us, incAngleMeas_rad, fitOrderServo);" << std::endl;
    // if(AnalogString != ""){
      std::cout << std::endl;
      std::cout << "% Pot: " << AnalogPath << std::endl;
      std::cout << "incAngleMeas_rad = [" << incAngleMeas_deg.transpose() << "] * d2r;" << std::endl;
      std::cout << "potValMeas_V = [" << potValMeas_V.transpose() << "];" << std::endl;
      std::cout << "fitOrderPot = 1; % Start with Linear, alter as required" << std::endl;
      std::cout << "[polyCoefPot] = PotCal(incAngleMeas_rad, potValMeas_V, fitOrderPot);" << std::endl;
    // }
    std::cout << std::endl;
  }

  return 0;
}
