# Configuration Fields

## Sensors
Sensor outputs are collected in the "/Sensors/" directory. The _Sensors_ JSON object is simply an array of sensors on the aircraft. The following lists two sensors connected to the FMU:
``` json
"Sensors": [
    { "Type": "Time", "Output": "Fmu/Time_us"},
    { "Type": "InputVoltage", "Output": "Fmu/Voltage/Input_V"}
]
```
Sensors on nodes are added by specifying the node address and listing sensors. For example, this adds PWM voltage and air data to the node at BFS-Bus address 3.
``` json
"Sensors": [
    { "Type": "Time", "Output": "Fmu/Time_us"},
    { "Type": "InputVoltage", "Output": "Fmu/Voltage/Input_V"},
    { "Type": "Node", "Address": 3,
      "Sensors": [
        { "Type": "PwmVoltage", "Output": "Power/WingServoVoltage_V"},
        { "Type": "Swift", "Output": "Pitot", "I2c": 1,
          "Static": {"Address": 20},
          "Differential": {"Address": 21, "Transducer": "AMS5915-0020-D"}
        }
      ]
    }
]
```

### Time
Time since the FMU booted, in microseconds. Specify the output location.
``` json
{ "Type": "Time", "Output": "Fmu/Time_us"}
```
### Voltages
Four different voltages are available:
   * "InputVoltage": the voltage input to the FMU
   * "RegulatedVoltage": the regulated voltage from the FMU voltage regulator
   * "PwmVoltage": the voltage of the PWM servo rail, this is available on the FMU or nodes.
   * "SbusVoltage": the voltage of the SBUS servo rail, this is available on the FMU or nodes.
Specify the output location. The output is appropriately scaled and converted to voltage.
``` json
{ "Type": "InputVoltage", "Output": "Fmu/Voltage/Input_V"}
```

### MPU-9250 IMU
MPU-9250 IMU data. The MPU-9250 is a 9-axis IMU and provides 3-axis gyro, accel, and magnetometer data. Gyro data is output in units of rad/s, accel data in units of m/s/s, and magnetometer data in units of uT. An MPU-9250 is integrated on the FMU and is referred to as "InternalMpu9250". Configurable items include a rotation matrix and digital low-pass filter bandwidth. The default orientation of the sensor is x-axis positive toward the BeagleBone Black ethernet port, y-axis positive right, and z-axis positive down.
``` json
{ "Type": "InternalMpu9250", "Output": "Fmu/Mpu9250", "Rotation": [-1,0,0,0,-1,0,0,0,1], "DLPF-Bandwidth": "20Hz"}
```
Additional MPU-9250 IMU's can be added, referred to as "Mpu9250". Additional configurable items include the I2C bus and address.
``` json
{ "Type": "Mpu9250", "Output": "Mpu9250", "I2c": 1, "Address": 104, "DLPF-Bandwidth": "20Hz"}
```

### BME-280 Static Pressure Sensor
Static pressure data from the BME-280 environmental sensor. Data is output in units of Pa. A BME-280 is integrated on the FMU and referred to as "InternalBme280". The only configurable item is the output location.
``` json
{ "Type": "InternalBme280", "Output": "Fmu/Bme280"}
```
Additional BME-280's can be added, referred to as "Bme280". Additional configurable items include the I2C bus and address.
``` json
{ "Type": "Bme280", "Output": "Fmu/Bme280", "I2c": 1, "Address": 118}
```
### SBUS Receiver
Measures and scales the 16 SBUS receiver channels to a -1 to +1 range. The input port is labeled SBUS RX on the FMU and nodes. Specify the output location.
``` json
{ "Type": "Sbus", "Output": "Sbus"}
```
### uBlox GNSS
uBlox GNSS data. The uBlox GNSS receiver must be setup to send the UBX-NAV-PVT packet at the desired rate using the [u-center software](https://www.u-blox.com/en/product/u-center). Configurable items include the UART port number, baudrate, and output location.
``` json
{ "Type": "uBlox", "Output": "uBlox", "Uart": 4, "Baud": 115200}
```
### Swift Air Data
Pitot-static data from the [Swift Air Data sensor](https://bolderflight.com/technical/#swift-air-data). Configurable items include the I2C bus, I2C addresses, the differential transducer type, and output location. Data is scaled appropriately and output in units of Pa.
``` json
{ "Type": "Swift", "Output": "Pitot", "I2c": 1,
          "Static": {"Address": 20},
          "Differential": {"Address": 21, "Transducer": "AMS5915-0020-D"}
```
### AMS-5915 Pressure Transducers
Pressure data from AMS-5915 pressure transducers. Configurable items include the I2C bus, I2C address, transducer type, and output location. Data is scaled appropriately and output in units of Pa.
``` json
{ "Type": "Ams5915", "Output": "diff_press", "I2c": 1, "Address": 29, "Transducer": "AMS5915-0020-D-B"}
```

### Analog Input
Data from the analog to digital converters. Configurable items include the channel number an a set of polynomial coefficients, given in descending order, to transform the measured voltage to engineering units.
``` json
{ "Type": "Analog", "Output": "Surf/posTE1L", "Channel": 0, "Calibration": [1.14680560, -1.81621446]}
```

## Sensor-Processing
Sensor processing outputs are collected in the "/Sensor-Processing/" directory. The _Sensor-Processing_ JSON object is an object that contains named JSON arrays of sensor-processing algorithms. Different sets of components are used; namely "Baseline" and "Test". The set defined in "Baseline" will always execute. Sets defined in the "Test" set will execute and engage via Mission Manager and Test Point definition. For example, the "Standard" definition is referenced in the "Baseline" set. "Test1" and "Test2" are referenced in the "Test" set. In this example "Standard" will always be running; if a test point is active and invokes "Test1" then "Test1" will run and be active. "Standard" will execute and copy its outputs in "/Sensor-Processing/", "Test" will execute and also place its outputs to "/Sensor-Processing/" overwriting elements if needed.
``` json

"Sensor-Processing": {
  "Fmu": "Standard",
  "Baseline": "Standard",
  "Test": ["Test1", "Test2"],

  "Def": {
    "Standard": [
      {"Type": "AGL", "Output": "hBaro_m",
        "Static-Pressure": ["/Sensors/Pitot/Static/Pressure_Pa"], "Initialization-Time": 10},
      {"Type": "IAS", "Output": "vIAS_ms",
        "Differential-Pressure": ["/Sensors/Pitot/Differential/Pressure_Pa"], "Initialization-Time": 10},
      {"Type": "EKF15StateINS", "Output": "INS",
        "Time": "/Sensors/Fmu/Time_us", "GPS": "/Sensors/uBlox", "IMU": "/Sensors/Fmu/Mpu9250"},
      {"Type": "MinCellVolt", "Output": "MinCellVolt_V",
        "Inputs": ["/Sensors/Fmu/Voltage/Input_V"],
        "NumCells": [3]}
    ],

    "Test1": [...],
    "Test2": [...]
  }
}
```

### Gain
Multiplies an input by a gain. Configurable items include the input, output, gain value, and optional limits, which saturate the output if defined.
```json
{ "Type": "Gain", "Input": "/Sensors/Sbus/Channels/7", "Output": "cmdMotor_nd", "Gain": 1.0, "Max": 1.0, "Min": 0.0}
```
### Sum
Sums a vector of inputs. Configurable items include a vector of inputs, the output, and optional limits, which saturate the output if defined.
```json
{ "Type": "Sum", "Inputs": ["/Control/cmdTE1L_alloc_rad", "/Control/cmdTE1L_flap_rad"], "Output": "cmdTE1L_rad", "Max": 0.436332, "Min": -0.436332}
```
### Product
Multiplies a vector of inputs. Configurable items include a vector of inputs, the output, and optional limits, which saturate the output if defined.
```json
 { "Type": "Product", "Inputs": ["/Control/cmdSurf_faultGain_nd", "/Control/cmdAilL_alloc_rad"], "Output": "cmdAilL_gained_rad"}
```
### Delay
Delays a signal by _N_ frames. Configurable items include the input, output, and number of frames to delay.
```json
{ "Type": "Delay", "Input": "/Control/cmdMotor_nom_nd", "Output": "cmdMotor_nd", "Delay_frames": 4}
```
### IAS
Computes indicated airspeed given a vector of differential pressure inputs. Configurable items include the differential pressure inputs, output, and initialization time - the time, in seconds, that should be used during initialization for estimating the pressure transducer biases. Data from all input sources will be averaged and used. Differential pressure input should be in units of Pa and output is in units of m/s.
```json
{ "Type": "IAS", "Output": "vIAS_ms", "Differential-Pressure": ["/Sensors/Pitot/Differential/Pressure_Pa"], "Initialization-Time": 10}
```
### AGL
Computes the altitude above ground level given a vector of static pressure inputs. Configurable items include the static pressure inputs, outputs, and initialization time - the time, in seconds, that should be used during initialization for estimating the AGL bias. Data from all input sources will be averaged and used. Static pressure input should be in units of Pa and output is in units of m.
```json
{ "Type": "AGL", "Output": "hBaro_m", "Static-Pressure": ["/Sensors/Pitot/Static/Pressure_Pa"], "Initialization-Time": 10}
```
### 15 State EKF INS
Uses a 15 state EKF with IMU and GNSS input to estimate inertial position, velocity, attitude, and gyro / accelerometer biases. Configurable items include the time source, GNSS data source, IMU data source, and output location.
```json
{ "Type": "EKF15StateINS", "Output": "INS", "Time": "/Sensors/Fmu/Time_us", "GPS": "/Sensors/uBlox", "IMU": "/Sensors/Fmu/Mpu9250"}
```

### Filter
Implements a general discrete time filter using the general filter difference equation. Configurable items include the input, output, a is a vector of denominator coefficients. a[0] scales all a and b coefficients if given. Denominator coefficients are optional and, if none are provided, a FIR filter is implemented. b is a vector of numerator coefficients. At least one feedforward coefficient must be given. The order of the filter is given by the length of the b and a vectors.

a[0]y[n] = b[0]x[n]+b[1]x[n-1]+b[2]x[n-2]+...-a[1]y[n-1]-a[2]y[n-2]-...

```json
{ "Type": "Filter", "Input": "/Sensor-Processing/GyroZ_rads", "Output": "cmdYawDamp_rps", "b": [-0.065, 0.065], "a": [1.0, -0.9418]}
```

### Minimum Cell Voltage
Computes the minimum cell voltage amongst a number of batteries. Configurable items include the inputs, output, and number of lipo cells.
```json
{ "Type": "MinCellVolt", "Output": "MinCellVolt_V", "Inputs": ["/Sensors/Fmu/Voltage/Input_V"], "NumCells": [3]}
```

## Control
Control outputs are collected in the "/Control/" directory. The _Control_ JSON object is an object that contains named JSON arrays of control sets, groups, and controllers, and components. The groups defined in the sets: "Fmu", "Baseline", and "Test" are managed uniquely. The "Fmu" set is parsed and sent to the FMU as a reversionary mode using a limited set of configurable blocks. The "Baseline" set defines controller groups; one "Baseline" group is always active in either and "armed" or "engaged" mode. The "Test" set defines an idependent set of controller groups. The Mission Manager controls execution of groups with in "Set". Only one "Test" group can be in any active state (either armed or engaged).

In this example the "Fmu" set references the "PilotDirect" group; the set "Baseline" references "PilotRate" and "PilotAttitudeMan" groups.

Controller Groups are defined in "GroupDef" and are split into Level:Controller pairs. Level names are used by the excitation system and should generally be consistently named across Groups.

In this example "PilotAttitudeMan" has Levels named; "Pilot", "SCAS", "Alloc", and "Eff".

Controller Definitions are defined in "ControlDef".

In this example "PilotAttitudeMan" references "Pilot-Att", whose Components are defined in "ControlDef".

Note that the "Baseline" set and the "Test" set will create independent instances of the groups and controllers used in their definition. So, in this example 2 completely independent instances of "PilotAttitudeMan" will be created; one in the "Baseline" set and one in the "Test". However, Controller instances are shared within a set; so, there is only one instance of "SCAS-Att" within the "Test" set.

```json

"Control": {
  "Fmu": ["PilotDirect"],
  "Baseline": ["PilotRate", "PilotAttitudeMan"],
  "Test": ["PilotAttitudeMan", "PilotAttitude17"],

  "GroupDef": {
    "PilotDirect": {"Pilot": "Direct"},
    "PilotRate": {"Pilot": "Pilot-Rate", "Alloc": "Allocator", "Eff": "Effector"},
    "PilotAttitudeMan": {
      "Pilot": "Pilot-Att", "SCAS": "SCAS-Att", "Alloc": "Allocator", "Eff": "Effector"},
    "PilotAttitude17": {
      "Route": "Guid-Ref17", "Pilot": "Pilot-Att", "Guid": "Guidance", "SCAS": "SCAS-Att", "Alloc": "Allocator", "Eff": "Effector"}
  },

  "ControlDef": {
    "Direct": [...],
    "Guid-Ref17": [...],
    "Guidance": [...],
    "Pilot-Rate": [...],
    "Pilot-Att": [...],
    "SCAS-Att": [...],
    "Allocator": [...],
    "Effector": [...]
  }
}

```
Controllers are defined in "ControlDef" as arrays of components. Components will be executed in the order defined.

Signal references are taken to be in the current contoller definition unless otherwise specified. In the example below: "Input": "cmdRoll_pid_rps" will pull values from "/Control/Test/SCAS-Att/cmdRoll_pid_rps", which was defined as an output within the same Controller.

Signal references that start with "../" will reference outputs of the next higher system. So, if "SCAS-Att" is referenced in the "Test" set, and defines an input as "../refPhi_rad", that signal value will get pulled from "/Control/Test/refPhi_rad". In this case, "/Control/Test/refPhi_rad" will be populated by a Controller that executed earlier in the Group.
```json

"SCAS-Att": [
  {"Type": "PID2", "Reference": "../refPhi_rad", "Feedback": "/Sensor-Processing/Roll_rad", "Output": "cmdRoll_pid_rps",
    "Kp": 0.64, "Ki": 0.20, "dt": 0.02, "Max": 1.0472, "Min": -1.0472},
  {"Type": "Gain", "Input": "/Sensor-Processing/GyroX_rads", "Output": "cmdRoll_damp_rps", "Gain": -0.070},
  {"Type": "Sum", "Inputs": ["cmdRoll_pid_rps", "cmdRoll_damp_rps"], "Output": "cmdRoll_rps",
    "Max": 1.0472, "Min": -1.0472},

  {"Type": "PID2", "Reference": "../refTheta_rad", "Feedback": "/Sensor-Processing/Pitch_rad", "Output": "cmdPitch_pid_rps",
    "Kp": 0.90, "Ki": 0.30, "dt": 0.02, "Max": 1.0472, "Min": -1.0472},
  {"Type": "Gain", "Input": "/Sensor-Processing/GyroY_rads", "Output": "cmdPitch_damp_rps", "Gain": -0.080},
  {"Type": "Sum", "Inputs": ["cmdPitch_pid_rps", "cmdPitch_damp_rps"], "Output": "cmdPitch_rps",
    "Max": 1.0472, "Min": -1.0472},

  {"Type": "Filter", "Input": "/Sensor-Processing/GyroZ_rads", "Output": "cmdYaw_damp_rps",
    "num": [-0.030, 0.030], "den": [1.0, -0.8919]},
  {"Type": "Sum", "Inputs": ["../refYaw_rps", "cmdYaw_damp_rps"], "Output": "cmdYaw_rps"}
]

```
### Constant
Outputs a constant value. Configurable items include the output location and the value.
```json
{ "Type": "Constant", "Output": "refPitch_bias_rps", "Constant": 0.25}
```
### Gain
Multiplies an input by a gain. Configurable items include the input, output, gain value, and optional limits, which saturate the output if defined.
```json
{ "Type": "Gain", "Input": "/Sensors/Sbus/Channels/7", "Output": "cmdMotor_nd", "Gain": 1.0, "Max": 1.0, "Min": 0.0},
```
### Sum
Sums a vector of inputs. Configurable items include a vector of inputs, the output, and optional limits, which saturate the output if defined.
```json
{ "Type": "Sum", "Inputs": ["/Control/cmdTE1L_alloc_rad", "/Control/cmdTE1L_flap_rad"], "Output": "cmdTE1L_rad", "Max": 0.436332, "Min": -0.436332}
```
### Product
Multiplies a vector of inputs. Configurable items include a vector of inputs, the output, and optional limits, which saturate the output if defined.
```json
 { "Type": "Product", "Inputs": ["/Control/cmdSurf_faultGain_nd", "/Control/cmdAilL_alloc_rad"], "Output": "cmdAilL_gained_rad"}
```
### Delay
Delays a signal by _N_ frames. Configurable items include the input, output, and number of frames to delay.
```json
{ "Type": "Delay", "Input": "/Control/cmdMotor_nom_nd", "Output": "cmdMotor_nd", "Delay_frames": 4}
```
### PID2
Implements a PID2 control law. Configurable items include:
  * Reference is the full path name of the reference signal.
  * Feedback is the full path name of the feedback signal.
  * Output gives a convenient name for the block (i.e. cmdPitch).
  * dt is either: the full path name of the sample time signal in seconds,
    or a fixed value sample time in seconds.
  * Tf is the time constant for the derivative filter.
    If a time constant is not specified, then no filtering is used.
  * Gains specifies the proportional, derivative, and integral gains.
  * Setpoint weights (b and c) optionally specifies the setpoint
    weights (proportional and derivative) used in the controller.
  * Limits (Max and Min) are optional.
  Data types for all input and output values are float.
```
{
  "Type": "PID2",
  "Reference": "ReferenceName",
  "Feedback": "FeedbackName",
  "Output": "OutputName",
  "dt": "dt" or X,
  "Kp": Kp, "Ki": Ki, "Kd": Kd, "Tf": Tf,
  "b": b, "c": c
  "Min": X, "Max": X
}
```
### PID
Implements a PID control law.
```
{
  "Type": "PID",
  "Reference": "ReferenceName",
  "Output": "OutputName",
  "dt": "dt" or X,
  "Kp": Kp, "Ki": Ki, "Kd": Kd, "Tf": Tf,
  "Min": X, "Max": X
}
```
Where:
  * Reference is the full path name of the reference signal.
  * Output gives a convenient name for the block (i.e. cmdPitch).
  * dt is either: the full path name of the sample time signal in seconds,
    or a fixed value sample time in seconds.
  * Tf is the time constant for the derivative filter.
    If a time constant is not specified, then no filtering is used.
  * Gains specifies the proportional, derivative, and integral gains.
  * Limits (Max and Min) are optional.
  Data types for all input and output values are float.

### State Space
Implements a state space control law.
```
{
  "Type": "SS",
  "Inputs": ["InputNames"],
  "Outputs": ["OutputNames"],
  "dt": "dt" or X,
  "A": [[X]],
  "B": [[X]],
  "C": [[X]],
  "D": [[X]],
  "Min": [X], "Max": [X]
}
```
  The implemented algorithm uses a discrete state space model, with variable sample time.
  The A and B matrices supplied are the continuous form, a simple zero-order hold is used to compute the discrete form.

  xDot = A*x + B*u;

  y = C*x + D*u;

  where:  

  Ad = (Ac*dt + I);

  Bd = B*dt;

  Thus, x[k+1] = Ad*x + Bd*u;

### Filter
Implements a general discrete time filter using the general filter difference equation. Configurable items include the input, output, a is a vector of denominator coefficients. a[0] scales all a and b coefficients if given. Denominator coefficients are optional and, if none are provided, a FIR filter is implemented. b is a vector of numerator coefficients. At least one feedforward coefficient must be given. The order of the filter is given by the length of the b and a vectors.

a[0]y[n] = b[0]x[n]+b[1]x[n-1]+b[2]x[n-2]+...-a[1]y[n-1]-a[2]y[n-2]-...

```json
{ "Type": "Filter", "Input": "/Sensor-Processing/GyroZ_rads", "Output": "cmdYawDamp_rps", "b": [-0.065, 0.065], "a": [1.0, -0.9418]}
```
### Pseudo Inverse
Implements a pseudo inverse control allocation.
```json
{"Type": "PseudoInverse",
  "Inputs": ["../cmdRoll_rps", "../cmdPitch_rps", "../cmdYaw_rps"],
  "Outputs": ["cmdElev_rad", "cmdRud_rad", "cmdAilR_rad", "cmdFlapR_rad", "cmdFlapL_rad", "cmdAilL_rad"],
  "Effectiveness": [
    [0.00000, -0.1418,-1.33413, -0.5634, 0.5634, 1.33413],
    [-2.2716, 0.00000, 0.06000, 0.05800, 0.05800, 0.06000],
    [0.00000,-1.59190, 0.00000, 0.00000, 0.00000, 0.00000]],
  "Min": [-0.436332, -0.261799, -0.436332, -0.436332, -0.436332, -0.436332],
  "Max": [0.436332, 0.261799, 0.436332, 0.436332, 0.436332, 0.436332]
}
```
Where:
  * Input gives the path of the allocator inputs / objectives (i.e. ../PitchMomentCmd)
  * Output gives the path of the allocator outputs / effector commands (i.e Elevator)
  * Effectiveness gives the control effectiveness (i.e. change in moment for a unit change in effector output)
    The order is NxM where M is the number of outputs / effectors and N is the number of inputs / objectives. So, for a situation with 3 objectives (i.e. pitch, roll, yaw moments) and 4 control surfaces, Effectiveness would be given as:
    "Effectiveness":[[RollEff_Surf0, RollEff_Surf1, RollEff_Surf2, RollEff_Surf3],
                     [PitchEff_Surf0, PitchEff_Surf1, PitchEff_Surf2, PitchEff_Surf3],
                     [YawEff_Surf0, YawEff_Surf1, YawEff_Surf2, YawEff_Surf3],]
  * Min and Max gives the upper and lower limits for each output / effector command.

### Latch
Latches the output to the initial input.
```
{
  "Type": "Latch",
  "Output": "OutputName",
  "Input": "InputName",
  }
}
```
Where:
   * Output gives a convenient name for the block (i.e. SpeedControl).
   * Input is the full path name of the input signal.
Data types for the input and output are both float.

## Excitation
Excitation outputs are collected in the "/Excitation/" directory. Similar to control laws, excitation groups can be defined consisting of multiple wave inputs, as defined in "ExciteDef". A group is applied PRIOR to execution of the defined controller level. Groups can be referenced in the mission manager test point structure, allowing different excitations for each pre-defined test point. For example, the following defines an excitation group consisting of multisine inputs.
```json
"Excitation": {
  "Time": "/Sensors/Fmu/Time_us",
  "ExciteDef": {
    "RTSM": {"Level": "Alloc", "Waveforms": [
      {"Wave": "OMS_1", "Signal": "/Control/Test/cmdRoll_rps", "Start-Time": 1, "Scale-Factor": 0.0698132},
      {"Wave": "OMS_2", "Signal": "/Control/Test/cmdPitch_rps", "Start-Time": 1, "Scale-Factor": 0.0698132},
      {"Wave": "OMS_3", "Signal": "/Control/Test/cmdYaw_rps", "Start-Time": 1, "Scale-Factor": 0.0698132}
    ]}
  },

  "WaveDef": {
    "OMS_1": {...},
    "OMS_2": {...},
    "OMS_3": {...}
  }
}

```

The basic waveforms are defined within "WaveDef", enabling them to be re-used on different signals, with different start times, and with different scale factors.
```json
"OMS_1": { "Type": "MultiSine", "Duration": 10,
  "Frequency": [0.62831853071795862,4.39822971502571,8.1681408993334639,11.938052083641214,15.707963267948969,19.477874452256717,23.247785636564469,27.017696820872224,30.787608005179976,34.557519189487721,38.32743037379548,42.097341558103231,45.867252742410983,49.637163926718735],
  "Phase": [6.1333837562683,3.7697783413786747,4.3326309124460112,5.7825002989603558,3.6123449608782874,3.8799569048390872,6.3167977328030709,4.6577199432535883,5.2790228241172983,8.05687689770693,7.08109780771917,2.4467121117324315,5.00303679294031,4.0729726069734049],
  "Amplitude": [1,1,1,1,1,1,1,1,1,1,1,1,1,1]
}
```

### Pulse
Adds a pulse to the signal for the specified duration
```
{
  "Type": "Pulse"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": X,
  "Scale-Factor": X
}
```
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of the pulse in seconds
   * Amplitude is the pulse amplitude
   * Scale factor is optional and provides another option to scale the output signal
### Doublet
Adds a doublet to the signal for the specified duration
```
{
  "Type": "Doublet"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": X
}
```
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 2 times the duration
   * Amplitude is the doublet amplitude
### Doublet 121
Adds a 1-2-1 doublet to the signal for the specified duration
```
{
  "Type": "Doublet121"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": X
}
```
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 4 times the duration
   * Amplitude is the doublet amplitude
### Doublet 3211
Adds a 3-2-1-1 doublet to the signal for the specified duration
```
{
  "Type": "Doublet3211"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": X
}
```
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 7 times the duration
   * Amplitude is the doublet amplitude
### Linear Chirp
Adds a linear chirp to the signal for the specified duration
```
{
  "Type": "LinearChirp"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": [start,end],
  "Frequency": [start,end]
}
```
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of the chirp
   * Amplitude is an array specifying the starting and ending chirp amplitude
   * Frequency is an array specifying the starting and ending frequency in rad/sec
### Log Chirp
Adds a log chirp to the signal for the specified duration
```
{
  "Type": "LogChirp"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": [start,end],
  "Frequency": [start,end]
}
```
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of the chirp
   * Amplitude is an array specifying the starting and ending chirp amplitude
   * Frequency is an array specifying the starting and ending frequency in rad/sec
### 1-Cos
Adds a 1-cosine signal for the specified duration
```
{
  "Type": "1-Cos"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Pause": X,
  "Amplitude": X
}
```
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of the 1-cos wave
   * Pause is the pause time at the peak of the 1-cos wave
### MultiSine
Adds a multisine to the signal for the specified duration
```
{
  "Type": "MultiSine"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": [X],
  "Frequency": [X],
  "Phase": [X]
}
```
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of the chirp
   * Amplitude, frequency, and phase are vectors specifying the amplitude,
     frequency, and phase of the multisine. They should all be the same length.
## Effector
Similar to Sensors, Effectors simply lists an array of all effectors on the vehicle. Effectors on nodes are added by specifying the node address and listing the effectors. For example, this specifies 3 effectors connected to the FMU (a motor and two servos) and 4 effectors connected to the node.
```json
"Effectors": [
  { "Type": "Motor", "Input": "/Control/cmdMotor_nd", "Channel": 0, "Calibration": [800, 1200], "Safed-Command": 0},
  { "Type": "Pwm", "Input": "/Control/cmdElev_rad", "Channel": 1, "Calibration": [ 0.0, 0.0, 945.56893619, 1532.92371906]},
  { "Type": "Pwm", "Input": "/Control/cmdRud_rad", "Channel": 2, "Calibration": [ -406.23428499, -72.84894531, -822.84212170, 1495.85430736]},
  { "Type": "Node", "Address": 3,
    "Effectors": [
      { "Type": "Pwm", "Input": "/Control/cmdAilR_rad", "Channel": 0, "Calibration": [ 0.0, 24.74931945, -877.03584524, 1501.05819261]},
      { "Type": "Pwm", "Input": "/Control/cmdFlapR_rad", "Channel": 2, "Calibration": [ 0.0, -167.40715677, 830.81057931, 1499.57954854]},
      { "Type": "Pwm", "Input": "/Control/cmdFlapL_rad", "Channel": 5, "Calibration": [ -280.29921208, 75.65078940, -855.43748599, 1442.51416948]},
      { "Type": "Pwm", "Input": "/Control/cmdAilL_rad", "Channel": 7, "Calibration": [ 0.0, -64.70244278, 828.18022105, 1519.68482092]}
    ]
  }
]
```

Three types of effectors are available: motor, PWM, and SBUS. All three have the input signal, channel number, and calibration as configurable items. The input signal is the source of the effector command and is typically an output from a control law. The channel number refers to the PWM or SBUS channel number the effector is connected to. Calibration is a vector of polynomial coefficients given in descending order, which converts the input signal from engineering units (i.e. trailing edge down angle value) to PWM or SBUS command. The motor effector type is the same as PWM, except it also includes a "Safed-Command", which specifies the value to be commanded when the throttle is safed.

## Mission Manager
The mission manager configures the switches used to define the vehicle's state along with a vector of test points. The switch definitions are:
  "Soc-Engage-Switch" - Modes: "Fmu" and "Soc" - used in FMU and SOC to dictate whether the FMU   or SOC is in active control.

  "Throttle-Safety-Switch" - Modes: "Safe" and "Engage" used in FMU to Disable Motor type effectors

  "Baseline-Select-Switch" - Modes: Groups within "Baseline" Control Set. Selects which Baseline controller is selected to be active.

  "Test-Mode-Switch" - Modes: "Standby", "Arm", "Engage" - Controls the run mode of the active Test Controller, as defined by the current Test Point.

  "Test-Select-Switch" - Modes: "Decrement", "Excite", "Increment" - Dictates the effect "Trigger-Switch" will have. Excitations will only execute with the switch in "Excite". Placing the switch in either the increment or decrement will stop the excitation. Changing the selected Test-Point imediately places the defined Sensor-Processing and Control groups active. The run mode depends on the current Test Run Mode as determined from "Test-Mode-Switch". So, if Test Mode is engaged and the test id is incremented the newly selected controller will imediately be engaged.

  "Trigger-Switch" - Modes: "Standby", "Trigger" - Either increment/decrement the test point index, or execute the selected test point excitation depending on the position of "Test-Select-Switch". Triggering with a excitation executing will stop the excitation, triggering again will restart it.


```json
"Mission-Manager": {
  "Soc-Engage-Switch": {"Source": "/Sensors/Sbus/Channels/0", "Gain": 1,
    "ModeSel": {"Fmu": -2, "Soc": 0}},
  "Throttle-Safety-Switch": {"Source": "/Sensors/Sbus/Channels/1", "Gain": 1,
    "ModeSel": {"Safe": -2, "Engage": 0.5}},
  "Test-Mode-Switch": {"Source": "/Sensors/Sbus/Channels/8", "Gain": 1,
    "ModeSel": {"Standby": -2, "Arm": -0.5, "Engage": 0.5}},
  "Test-Select-Switch": {"Source": "/Sensors/Sbus/Channels/9", "Gain": 1,
    "ModeSel": {"Decrement": -2, "Excite": -0.5, "Increment": 0.5}},
  "Trigger-Switch": {"Source": "/Sensors/Sbus/Channels/10", "Gain": 1,
    "ModeSel": {"Standby": -2, "Trigger": 0.0}},

  "Baseline-Select-Switch": {"Source": "/Sensors/Sbus/Channels/11", "Gain": 1,
    "ControlSel": {"PilotRate": -2, "PilotAttitudeMan": -0.5, "PilotAttitudeMan": 0.5}},

  "Test-Points": []
}
```
Test points can be defined for the sensor-processing, control law, and excitation to use. This is useful to predefine all research test points in a flight.
```json
"Test-Points": [
  { "Test-ID": "0", "Sensor-Processing": "Standard", "Control": "PilotAttitudeMan", "Excitation": "RTSM"},
  { "Test-ID": "1", "Sensor-Processing": "Standard", "Control": "PilotAttitudeMan", "Excitation": "RTSM_Long"},
  { "Test-ID": "2", "Sensor-Processing": "Standard", "Control": "PilotAttitudeMan", "Excitation": "RTSM_Large"}
]
```
