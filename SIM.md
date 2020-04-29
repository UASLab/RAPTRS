
# Principles
- JSBSim is the NL simulation source
	- Attempt to wrap with Python with the provided Python bindings for JSBSim
- A fake version of the FMU emulates the behavior of the FMU
	- Somthing like: "SimFmu.py"
	- Priority on simulation and communication; rather than any control aspects the real FMU provides
- Minimal intrusion to SOC code
	- SOC code is what we're really testing
	- Preserve as much of the SOC side of the SOC-FMU comm as possible

- SOC Modification for SIL/HIL testing:
	- If "JSBSim" is in the provided in the aircraft .json config file, then the SOC opens the defined port as the FMU connection:
	> 	"JSBSim": { 
			"Model": "UltraStick25e",
			"SimFmuPort": <port on SOC side>,
			"SimFmuBaud": 1500000
	> 		},

# Simulation modes:

## HIL (Hardware in the Loop)
- Run the SOC code (./flight) on a BBB attached to a PC which is running JSBSim

- Comments:
	- The BBB-PC USB connection is already used as the Ethernet connection
		- Could be used as a UDP conduit (Socket or TCP...)
			- Need SOC serial to UDP bridge on the SOC side
			- Need UDP to serial bridge on the SimFmu.py side (or SimFmu.py is just UDP)
	- An FTDI-USB could be used on a BBB-UART


## SIL (Software in the Loop)
- Run the SOC code (./flight_amd64) on the PC along side JSBSim

- Comments:
	- Makes sense to have SimFmu.py as similar to the HIL case as possible
	- For SIL there is also the option of letting the sim run as fast as possible (or at different rates) (assuming a pilot isn't in the loop). This should just be handled on the Python side only, the SOC has no internal sense of time and will run as fast as possible.

# Simulated/Fake FMU Flow (Python):
### Config:
1. Read each config message, Ack each
2. Map JSBSim properties and faked sources in order to populate messages
3. Setup JSBSim to start (trim, etc. - may need to step it a few times to get sensor data fully populated)

### Run:
1. Read Sensors (JSBSim source, or otherwise faked)
2. Send Sensor messages to SOC (including joystick)
3. Read effector messages from SOC
4. Send effector commands (JSBSim)
5. Step Simulation (X times, for 20ms worth)
6. Wait until 20ms (from Sensor read) - **probably a lot of jitter**

- Note: Need to think about how to "fake" some of the data that isn't readily in JSBSim.
- Note: https://www.pygame.org/ should work to provide a joystick source directly to Python

## Normal FMU Flow (Just for reference... ):
### Config:
1. Read each config message, Ack each
2. Configure via Nodes, devices, ect.
3. Start all Nodes, devices, ect. (upon first entering Run I think)
### Run:
1. Read Sensors (self and Nodes)
2. Send Sensor messages to SOC
3. Run control
4. Read effector messages from SOC
5. Send effector commands (self and Nodes)
6. Wait until 18ms (from Read Sensors start)
7. Send effector trigger

