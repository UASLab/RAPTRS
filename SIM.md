Simulation modes:

HIL (Hardware in the Loop) Run the SOC code on a BBB attached to a PC which is running JSBSim If "JSBSim" is in the provided .json config, then the SOC should open the port as provided in the config: "JSBSim": { "Model": "UltraStick25e", "SimFmuPort": /dev/ttyACM0, "SimFmuBaud": 1500000, },

On the SOC the port is hardcoded in hardware-defs.h as: "/dev/ttyO4" @ 1500000 baud FlightManagementUnit::Begin() uses those values to setup the FMU serial comm flightcode.cpp calls: Fmu.Begin();

Recommend: change: FlightManagementUnit::Begin() to take Port and Baud arguments, default to the hardcoded values change: flightcode.cpp to read the config and pass the Port and Baud arguments

 "SimFmu.py" needs to get the same SimFmuPort defintion (either by getting the JSON or via command line arg)
SIL (Software in the Loop) Run the SOC code (make flight_amd64) on the PC along side JSBSim On Linux I think this would work the same as the HIL mode: Python and flight_amd64 application would chat via a common serial device On Windows we'd need to figure out how make it work.

For SIL there is also the option of letting the sim run as fast as possible (or at different rates) (assuming a pilot isn't in the loop) this should just be handled on the Python side.

In both sim modes: JSBSim runs in Python A fake version of the FMU "SimFmu.py" emulates the behavior of the FMU

Sim FMU:

Read Sensors (JSBSim source, or otherwise faked)
Send Sensor messages to SOC
Read effector messages from SOC
Send effector commands (JSBSim)
Step Simulation (X times, for 20ms worth)
wait until 20ms (from Sensor read)
Note: Need to think about how to "fake" some of the data that isn't readily in JSBSim. I've seen a few examples of OpenTx being read as a joystick in Python.

Just for reference... Normal FMU:

Read Sensors (self and Nodes)
Send Sensor messages to SOC
Run control
Read effector messages from SOC
Send effector commands (self and Nodes)
wait until 18ms
Send effector trigger
