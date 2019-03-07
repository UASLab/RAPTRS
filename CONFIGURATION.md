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
Sensor processing outputs are collected in the "/Sensor-Processing/" directory. The _Sensor-Processing_ JSON object is an object that contains named JSON arrays of sensor-processing algorithms. For example, this defines two arrays of algorithms, one named "Baseline" and the other "Research". The mission manager can select a different array depending on the test point; with this approach you can test research sensor-processing algorithms with baseline algorithms available as a reversionary mode.
``` json
"Sensor-Processing": {
    "Baseline": [
      { "Type": "AGL", "Output": "hBaro_m",
        "Static-Pressure": ["/Sensors/Pitot/Static/Pressure_Pa"], "Initialization-Time": 10}
    ],
    "Research": [
      { "Type": "AGL", "Output": "hBaro_m",
        "Static-Pressure": ["/Sensors/Fmu/Bme280"], "Initialization-Time": 10}
    ]
  }
```
### Gain
Multiplies an input by a gain. Configurable items include the input, output, gain value, and optional limits, which saturate the output if defined.
```json
{ "Type": "Gain", "Input": "/Sensors/Sbus/Channels/7", "Output": "cmdMotor_nd", "Gain": 1.0, "Limits": {"Upper": 1.0, "Lower": 0.0}},
```
### Sum
Sums a vector of inputs. Configurable items include a vector of inputs, the output, and optional limits, which saturate the output if defined.
```json
{ "Type": "Sum", "Inputs": ["/Control/cmdTE1L_alloc_rad", "/Control/cmdTE1L_flap_rad"], "Output": "cmdTE1L_rad", "Limits": {"Upper": 0.436332, "Lower": -0.436332}
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
### If
Outputs a 0 or 1 depending on input and threshold values - if the input is greater than the threshold, the output is 1 otherwise, it's 0. Configurable items include the input, output, and threshold value.
```json
{ "Type": "If", "Input": "/Sensors/Sbus/Channels/7", "Output": "gear", "Threshold": 0.5}
```
### Minimum Cell Voltage
Computes the minimum cell voltage amongst a number of batteries. Configurable items include the inputs, output, and number of lipo cells.
```json
{ "Type": "MinCellVolt", "Output": "MinCellVolt_V", "Inputs": ["/Sensors/Fmu/Voltage/Input_V"], "NumCells": [3]}
```
## Control
Control outputs are collected in the "/Control/" directory. The _Control_ JSON object is an object that contains named JSON arrays of sensor-processing algorithms. The "Fmu" named control law is run on the FMU as a reversionary mode using a limited set of configurable blocks. The "Soc" control laws contain a vector naming several control laws that can be run. These can be selected by the mission manager enabling launch, landing, and test points to use individual control laws to facilitate research and provide reversionary modes. The following defines a control law named "PilotDirect" to run on the FMU and a "PilotRate" and "PilotAttitude" control laws to run on the SOC.
```json
"Control": {
    "Fmu": "PilotDirect",
    "Soc": ["PilotRate", "PilotAttitude"],
```
Following this initial declaration, each control law is defined. First, each control law can have one or more named levels. And each level constsists of a name and a vector of components. For example, "PilotDirect" is only one level deep and consists of a series of gain components mapping SBUS input commands (i.e. pilot stick inputs) to effector outputs.
```json
"PilotDirect": [
      { "Level": "SCAS-Rate",
        "Components": [
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/7", "Output": "cmdMotor_nd", "Gain": 1, "Limits": {"Upper": 1, "Lower": 0}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/3", "Output": "cmdAilL_rad", "Gain": 0.45, "Limits": {"Upper": 0.45, "Lower": -0.45}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/3", "Output": "cmdAilR_rad", "Gain": -0.45, "Limits": {"Upper": 0.45, "Lower": -0.45}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/4", "Output": "cmdElev_rad", "Gain": -0.45, "Limits": {"Upper": 0.45, "Lower": -0.45}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/5", "Output": "cmdRud_rad", "Gain": -0.45, "Limits": {"Upper": 0.45, "Lower": -0.45}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/6", "Output": "cmdFlapL_rad", "Gain": 0.45, "Limits": {"Upper": 0.45, "Lower": -0.45}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/6", "Output": "cmdFlapR_rad", "Gain": 0.45, "Limits": {"Upper": 0.45, "Lower": -0.45}}
        ]
      }
    ]
```
Control laws can have multiple levels. Each level is run in order and the outputs from one level can be the inputs to the next. This allows defining cascaded control laws. For example, "PilotRate" uses two levels - the first to map pilot stick inputs to aircraft rate commands and the second to perform pseudo inverse control allocation to determine effector positions to meet those commands.
```json
"PilotRate": [
      { "Level": "SCAS-Rate",
        "Components": [
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/7", "Output": "cmdMotor_nd", "Gain": 1, "Limits": {"Upper": 1, "Lower": 0}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/3", "Output": "cmdRoll_rps", "Gain": 1.0472, "Limits": {"Upper": 1.0472, "Lower": -1.0472}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/4", "Output": "cmdPitch_rps", "Gain": 1.0472, "Limits": {"Upper": 1.0472, "Lower": -1.0472}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/5", "Output": "cmdYaw_rps", "Gain": 0.523599, "Limits": {"Upper": 0.523599, "Lower": -0.523599}},
          { "Type": "Gain", "Input": "/Sensors/Sbus/Channels/6", "Output": "refFlap_rad", "Gain": 0.436332, "Limits": {"Upper": 0.436332, "Lower": -0.436332}}
        ]
      },

      { "Level": "Allocator",
        "Components": [
          { "Type": "PseudoInverse",
            "Inputs": ["/Control/cmdRoll_rps", "/Control/cmdPitch_rps", "/Control/cmdYaw_rps"],
            "Outputs": ["cmdElev_rad", "cmdRud_rad", "cmdAilR_rad", "cmdFlapR_alloc_rad", "cmdFlapL_alloc_rad", "cmdAilL_rad"],
            "Effectiveness": [
              [0.00000, -0.1418,-1.33413, -0.5634, 0.5634, 1.33413],
              [-2.2716, 0.00000, 0.06000, 0.05800, 0.05800, 0.06000],
              [0.00000,-1.59190, 0.00000, 0.00000, 0.00000, 0.00000]],
            "Limits": {
              "Lower": [-0.436332, -0.261799, -0.436332, -0.436332, -0.436332, -0.436332],
              "Upper": [0.436332, 0.261799, 0.436332, 0.436332, 0.436332, 0.436332]
            }
          }
        ]
      }
```
### Constant
Outputs a constant value. Configurable items include the output location and the value.
```json
{ "Type": "Constant", "Output": "refPitch_bias_rps", "Constant": 0.25}
```
### Gain
Multiplies an input by a gain. Configurable items include the input, output, gain value, and optional limits, which saturate the output if defined.
```json
{ "Type": "Gain", "Input": "/Sensors/Sbus/Channels/7", "Output": "cmdMotor_nd", "Gain": 1.0, "Limits": {"Upper": 1.0, "Lower": 0.0}},
```
### Sum
Sums a vector of inputs. Configurable items include a vector of inputs, the output, and optional limits, which saturate the output if defined.
```json
{ "Type": "Sum", "Inputs": ["/Control/cmdTE1L_alloc_rad", "/Control/cmdTE1L_flap_rad"], "Output": "cmdTE1L_rad", "Limits": {"Upper": 0.436332, "Lower": -0.436332}
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
   * Output gives a convenient name for the block (i.e. PitchControl).
   * Reference is the full path name of the reference signal.
   * Feedback is the full path name of the feedback signal.
   * Sample-Time is either: the full path name of the sample time signal in seconds,
     or a fixed value sample time in seconds.
   * Time-Constant is the time constant for the derivative filter.
     If a time constant is not specified, then no filtering is used.
   * Gains specifies the proportional, derivative, and integral gains.
   * Setpoint weights optionally specify the proportional and derivative setpoint
     weights used in the filter.
   * Limits are optional and saturate the output if defined.
Data types for all input and output values are float.
```
{
  "Type": "PID2",
  "Output": "OutputName",
  "Reference": "ReferenceName",
  "Feedback": "FeedbackName",
  "Sample-Time": "SampleTime" or X,
  "Time-Constant": X,
  "Gains": {
    "Proportional": Kp,
    "Integral": Ki,
    "Derivative": Kd,
  },
  "Setpoint-Weights": {
    "Proportional": b,
    "Derivative": c
  },
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
```
### PID
Implements a PID control law. 
```
{
  "Type": "PID",
  "Output": "OutputName",
  "Reference": "ReferenceName",
  "Sample-Time": "SampleTime" or X,
  "Time-Constant": X,
  "Gains": {
    "Proportional": Kp,
    "Integral": Ki,
    "Derivative": Kd,
  },
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
```
Where:
   * Output gives a convenient name for the block (i.e. PitchControl).
   * Reference is the full path name of the reference signal.
   * Sample-Time is either: the full path name of the sample time signal in seconds,
     or a fixed value sample time in seconds.
   * Time-Constant is the time constant for the derivative filter.
     If a time constant is not specified, then no filtering is used.
   * Gains specifies the proportional derivative and integral gains.
   * Limits are optional and saturate the output if defined.
Data types for all input and output values are float.
### State Space
Implements a state space control law.
```
{
  "Type": "SS",
  "Name": "Name",
  "Inputs": ["InputNames"],
  "Outputs": ["OutputNames"],
  "Sample-Time": "SampleTime" or X,
  "A": [[X]],
  "B": [[X]],
  "C": [[X]],
  "D": [[X]],
  "Limits": {
    "Upper": [X],
    "Lower": [X]
  }
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
{ "Type": "PseudoInverse",
            "Inputs": ["/Control/cmdVert_nd", "/Control/cmdRoll_rps", "/Control/cmdPitch_rps", "/Control/cmdYaw_rps"],
            "Outputs": ["cmdMotorFR_nd", "cmdMotorAL_nd", "cmdMotorFL_nd", "cmdMotorAR_nd"],
            "Effectiveness": [
              [ 0.25000, 0.25000, 0.25000, 0.25000],
              [-0.25000, 0.25000, 0.25000,-0.25000],
              [ 0.25000,-0.25000, 0.25000,-0.25000],
              [ 0.25000, 0.25000,-0.25000,-0.25000]],
            "Limits": {
              "Lower": [0.0, 0.0, 0.0, 0.0],
              "Upper": [1.0, 1.0, 1.0, 1.0]
            }
```
Where:
   * Input gives the full path of the allocator inputs / objectives (i.e. /Control/PitchMomentCmd)
   * Output gives the relative path of the allocator outputs / effector commands (i.e Elevator)
   * Effectiveness gives the control effectiveness (i.e. change in moment for a unit change in effector output)
     The order is MxN where M is number of inputs / objectives and N isthe number of outputs / effectors. So,
     for a situation with 3 objectives (i.e. pitch, roll, yaw moments) and 7 control surfaces, Effectiveness would
     be given as:
     ```
     "Effectiveness":[[PitchEff_Surf0,...,PitchEff_Surf6],
                      [RollEff_Surf0,...,RollEff_Surf6],
                      [YawEff_Surf0,...,YawEff_Surf6]]
   * Limits gives the upper and lower limits for each output / effector command.
### TECS
Implements a Total Energy Control System control law.
```
{
  "Type": "Tecs",
  "mass_kg": x,
  "weight_bal": x,
  "max_mps": x,
  "min_mps": x,
  "RefSpeed": "RefSpeed",
  "RefAltitude": "RefAltitude",
  "FeedbackSpeed": "FeedbackSpeed",
  "FeedbackAltitude": "FeedbackAltitude",
  "OutputTotal": "OutputTotal",
  "OutputDiff": "OutputDiff"
}
```
Where:
   * mass_kg is the total aircraft weight in kg
   * weight_bal is a value = [0.0 - 2.0] with 1.0 being a good starting point.
     0.0 = elevator controls speed only, 2.0 = elevator controls altitude only
   * min_mps: the system will not command a pitch angle that causes the
     airspeed to drop below min_mps, even with zero throttle.
   * max_mps: the system will not command a combination of pitch and throttle
     that will cause the airspeed to exceed this value
   * In either case it is possible to momentarily bust these limits, but the
     system will always be driving the airspeed back within the specified limits

Data types for all input and output values are float.
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
Excitation outputs are collected in the "/Excitation/" directory. Similar to control laws, excitation groups can be defined consisting of multiple excitation inputs. Each group can define multiple excitation levels, specifying the control law level where the excitation should be inserted, enabling excitations for reference commands and effector signals. Each group can be referenced in the mission manager test point structure, allowing different excitations for each pre-defined test point. For example, the following defines an excitation group consisting of multisine inputs on several reference commands.
```
{ "Name": "Aero_Med", "Components": [
        { "Level": "Allocator", "Components": [
          { "Waveform": "OMS_1", "Signal": "/Control/cmdRoll_rps", "Start-Time": 1, "Scale-Factor": 0.0698132},
          { "Waveform": "OMS_2", "Signal": "/Control/cmdPitch_rps", "Start-Time": 1, "Scale-Factor": 0.0698132},
          { "Waveform": "OMS_3", "Signal": "/Control/cmdYaw_rps", "Start-Time": 1, "Scale-Factor": 0.0698132}
        ]}
      ]}
```

The basic waveforms are defined separately, enabling them to be re-used on different surfaces with different start times and scale factors.
```
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

## Mission Manager
