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
"Sensors": [
    { "Type": "Time", "Output": "Fmu/Time_us"},
    { "Type": "InputVoltage", "Output": "Fmu/Voltage/Input_V"}
]
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

### AGL

### Pitot Static

### 15 State EKF INS

### Filter

### If

### Minimum Cell Voltage


## Control

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

### PID

### State Space

### Filter

### Pseudo Inverse

### TECS

### Latch

## Excitation

### Doublet

### Doublet 121

### Doublet 3211

### Linear Chirp

### Log Chirp

### 1-Cos

### MultiSine

## Mission Manager
