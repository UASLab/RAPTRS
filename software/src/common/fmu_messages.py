import struct

# Message id constants
command_mode_id = 10
command_effectors_id = 11
config_ack_id = 20
config_basic_id = 21
config_mpu9250_id = 22
config_bme280_id = 23
config_ublox_id = 24
config_ams5915_id = 25
config_swift_id = 26
config_analog_id = 27
config_effector_id = 28
config_mission_id = 29
config_control_gain_id = 30
data_time_id = 40
data_mpu9250_short_id = 41
data_mpu9250_id = 42
data_bme280_id = 43
data_ublox_id = 44
data_ams5915_id = 45
data_swift_id = 46
data_sbus_id = 47
data_analog_id = 48
data_compound_id = 49

# Constants
num_effectors = 16  # number of effector channels
max_calibration = 4  # maximum nubmer of calibration coefficients
accel_scale = 208.82724  # 1 / (9.807(g) * 16 / 32767.5) (+/-16g)
gyro_scale = 938.71973  # 1 / (2000.0f/32767.5f * d2r (+/-2000deg/sec)
mag_scale = 300  # fits range

# Enums
sensor_type_time = 0
sensor_type_input_voltage = 1
sensor_type_regulated_voltage = 2
sensor_type_pwm_voltage = 3
sensor_type_sbus_voltage = 4
sensor_type_internal_bme280 = 5
sensor_type_sbus = 6
effector_type_motor = 0
effector_type_pwm = 1
effector_type_sbus = 2

# Message: command_mode
# Id: 10
class command_mode():
    id = 10
    _pack_string = "<b"

    def __init__(self, msg=None):
        # public fields
        self.mode = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.mode)
        return msg

    def unpack(self, msg):
        (self.mode,) = struct.unpack(self._pack_string, msg)

# Message: command_effectors
# Id: 11
class command_effectors():
    id = 11
    _pack_string = "<Bhhhhhhhhhhhhhhhh"

    def __init__(self, msg=None):
        # public fields
        self.num_active = 0
        self.command = [0.0] * num_effectors
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.num_active,
                          int(round(self.command[0] * 32767)),
                          int(round(self.command[1] * 32767)),
                          int(round(self.command[2] * 32767)),
                          int(round(self.command[3] * 32767)),
                          int(round(self.command[4] * 32767)),
                          int(round(self.command[5] * 32767)),
                          int(round(self.command[6] * 32767)),
                          int(round(self.command[7] * 32767)),
                          int(round(self.command[8] * 32767)),
                          int(round(self.command[9] * 32767)),
                          int(round(self.command[10] * 32767)),
                          int(round(self.command[11] * 32767)),
                          int(round(self.command[12] * 32767)),
                          int(round(self.command[13] * 32767)),
                          int(round(self.command[14] * 32767)),
                          int(round(self.command[15] * 32767)))
        return msg

    def unpack(self, msg):
        (self.num_active,
         self.command[0],
         self.command[1],
         self.command[2],
         self.command[3],
         self.command[4],
         self.command[5],
         self.command[6],
         self.command[7],
         self.command[8],
         self.command[9],
         self.command[10],
         self.command[11],
         self.command[12],
         self.command[13],
         self.command[14],
         self.command[15]) = struct.unpack(self._pack_string, msg)
        self.command[0] /= 32767
        self.command[1] /= 32767
        self.command[2] /= 32767
        self.command[3] /= 32767
        self.command[4] /= 32767
        self.command[5] /= 32767
        self.command[6] /= 32767
        self.command[7] /= 32767
        self.command[8] /= 32767
        self.command[9] /= 32767
        self.command[10] /= 32767
        self.command[11] /= 32767
        self.command[12] /= 32767
        self.command[13] /= 32767
        self.command[14] /= 32767
        self.command[15] /= 32767

# Message: config_ack
# Id: 20
class config_ack():
    id = 20
    _pack_string = "<BB"

    def __init__(self, msg=None):
        # public fields
        self.ack_id = 0
        self.ack_subid = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.ack_id,
                          self.ack_subid)
        return msg

    def unpack(self, msg):
        (self.ack_id,
         self.ack_subid) = struct.unpack(self._pack_string, msg)

# Message: config_basic
# Id: 21
class config_basic():
    id = 21
    _pack_string = "<BB"

    def __init__(self, msg=None):
        # public fields
        self.sensor = 0
        self.output = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.sensor,
                          len(self.output))
        msg += str.encode(self.output)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.sensor,
         self.output_len) = struct.unpack(self._pack_string, msg)
        self.output = extra[:self.output_len].decode()
        extra = extra[self.output_len:]

# Message: config_mpu9250
# Id: 22
class config_mpu9250():
    id = 22
    _pack_string = "<BBfffffffffbBBBBBBBBB"

    def __init__(self, msg=None):
        # public fields
        self.internal = False
        self.SRD = 0
        self.orientation = [0.0] * 9
        self.DLPF_bandwidth_hz = 0
        self.output = ""
        self.use_spi = False
        self.spi_bus = 0
        self.cs_pin = 0
        self.mosi_pin = 0
        self.miso_pin = 0
        self.sck_pin = 0
        self.i2c_bus = 0
        self.i2c_addr = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.internal,
                          self.SRD,
                          self.orientation[0],
                          self.orientation[1],
                          self.orientation[2],
                          self.orientation[3],
                          self.orientation[4],
                          self.orientation[5],
                          self.orientation[6],
                          self.orientation[7],
                          self.orientation[8],
                          self.DLPF_bandwidth_hz,
                          len(self.output),
                          self.use_spi,
                          self.spi_bus,
                          self.cs_pin,
                          self.mosi_pin,
                          self.miso_pin,
                          self.sck_pin,
                          self.i2c_bus,
                          self.i2c_addr)
        msg += str.encode(self.output)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        self.orientation_len = [0] * 9
        (self.internal,
         self.SRD,
         self.orientation[0],
         self.orientation[1],
         self.orientation[2],
         self.orientation[3],
         self.orientation[4],
         self.orientation[5],
         self.orientation[6],
         self.orientation[7],
         self.orientation[8],
         self.DLPF_bandwidth_hz,
         self.output_len,
         self.use_spi,
         self.spi_bus,
         self.cs_pin,
         self.mosi_pin,
         self.miso_pin,
         self.sck_pin,
         self.i2c_bus,
         self.i2c_addr) = struct.unpack(self._pack_string, msg)
        self.output = extra[:self.output_len].decode()
        extra = extra[self.output_len:]

# Message: config_bme280
# Id: 23
class config_bme280():
    id = 23
    _pack_string = "<BBBBBBBBB"

    def __init__(self, msg=None):
        # public fields
        self.use_spi = False
        self.spi_bus = 0
        self.cs_pin = 0
        self.mosi_pin = 0
        self.miso_pin = 0
        self.sck_pin = 0
        self.i2c_bus = 0
        self.i2c_addr = 0
        self.output = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.use_spi,
                          self.spi_bus,
                          self.cs_pin,
                          self.mosi_pin,
                          self.miso_pin,
                          self.sck_pin,
                          self.i2c_bus,
                          self.i2c_addr,
                          len(self.output))
        msg += str.encode(self.output)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.use_spi,
         self.spi_bus,
         self.cs_pin,
         self.mosi_pin,
         self.miso_pin,
         self.sck_pin,
         self.i2c_bus,
         self.i2c_addr,
         self.output_len) = struct.unpack(self._pack_string, msg)
        self.output = extra[:self.output_len].decode()
        extra = extra[self.output_len:]

# Message: config_ublox
# Id: 24
class config_ublox():
    id = 24
    _pack_string = "<BLB"

    def __init__(self, msg=None):
        # public fields
        self.uart = 0
        self.baud = 0
        self.output = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.uart,
                          self.baud,
                          len(self.output))
        msg += str.encode(self.output)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.uart,
         self.baud,
         self.output_len) = struct.unpack(self._pack_string, msg)
        self.output = extra[:self.output_len].decode()
        extra = extra[self.output_len:]

# Message: config_ams5915
# Id: 25
class config_ams5915():
    id = 25
    _pack_string = "<BBBB"

    def __init__(self, msg=None):
        # public fields
        self.i2c_bus = 0
        self.i2c_addr = 0
        self.transducer = ""
        self.output = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.i2c_bus,
                          self.i2c_addr,
                          len(self.transducer),
                          len(self.output))
        msg += str.encode(self.transducer)
        msg += str.encode(self.output)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.i2c_bus,
         self.i2c_addr,
         self.transducer_len,
         self.output_len) = struct.unpack(self._pack_string, msg)
        self.transducer = extra[:self.transducer_len].decode()
        extra = extra[self.transducer_len:]
        self.output = extra[:self.output_len].decode()
        extra = extra[self.output_len:]

# Message: config_swift
# Id: 26
class config_swift():
    id = 26
    _pack_string = "<BBBBB"

    def __init__(self, msg=None):
        # public fields
        self.i2c_bus = 0
        self.static_i2c_addr = 0
        self.diff_i2c_addr = 0
        self.diff_transducer = ""
        self.output = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.i2c_bus,
                          self.static_i2c_addr,
                          self.diff_i2c_addr,
                          len(self.diff_transducer),
                          len(self.output))
        msg += str.encode(self.diff_transducer)
        msg += str.encode(self.output)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.i2c_bus,
         self.static_i2c_addr,
         self.diff_i2c_addr,
         self.diff_transducer_len,
         self.output_len) = struct.unpack(self._pack_string, msg)
        self.diff_transducer = extra[:self.diff_transducer_len].decode()
        extra = extra[self.diff_transducer_len:]
        self.output = extra[:self.output_len].decode()
        extra = extra[self.output_len:]

# Message: config_analog
# Id: 27
class config_analog():
    id = 27
    _pack_string = "<BffffB"

    def __init__(self, msg=None):
        # public fields
        self.channel = 0
        self.calibration = [0.0] * max_calibration
        self.output = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.channel,
                          self.calibration[0],
                          self.calibration[1],
                          self.calibration[2],
                          self.calibration[3],
                          len(self.output))
        msg += str.encode(self.output)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        self.calibration_len = [0] * max_calibration
        (self.channel,
         self.calibration[0],
         self.calibration[1],
         self.calibration[2],
         self.calibration[3],
         self.output_len) = struct.unpack(self._pack_string, msg)
        self.output = extra[:self.output_len].decode()
        extra = extra[self.output_len:]

# Message: config_effector
# Id: 28
class config_effector():
    id = 28
    _pack_string = "<BBBfffff"

    def __init__(self, msg=None):
        # public fields
        self.effector = 0
        self.input = ""
        self.channel = 0
        self.calibration = [0.0] * max_calibration
        self.safed_command = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.effector,
                          len(self.input),
                          self.channel,
                          self.calibration[0],
                          self.calibration[1],
                          self.calibration[2],
                          self.calibration[3],
                          self.safed_command)
        msg += str.encode(self.input)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        self.calibration_len = [0] * max_calibration
        (self.effector,
         self.input_len,
         self.channel,
         self.calibration[0],
         self.calibration[1],
         self.calibration[2],
         self.calibration[3],
         self.safed_command) = struct.unpack(self._pack_string, msg)
        self.input = extra[:self.input_len].decode()
        extra = extra[self.input_len:]

# Message: config_mission
# Id: 29
class config_mission():
    id = 29
    _pack_string = "<BBff"

    def __init__(self, msg=None):
        # public fields
        self.switch_name = ""
        self.source = ""
        self.gain = 1.0
        self.threshold = 0.5
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          len(self.switch_name),
                          len(self.source),
                          self.gain,
                          self.threshold)
        msg += str.encode(self.switch_name)
        msg += str.encode(self.source)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.switch_name_len,
         self.source_len,
         self.gain,
         self.threshold) = struct.unpack(self._pack_string, msg)
        self.switch_name = extra[:self.switch_name_len].decode()
        extra = extra[self.switch_name_len:]
        self.source = extra[:self.source_len].decode()
        extra = extra[self.source_len:]

# Message: config_control_gain
# Id: 30
class config_control_gain():
    id = 30
    _pack_string = "<BBBfBff"

    def __init__(self, msg=None):
        # public fields
        self.level_name = ""
        self.input = ""
        self.output = ""
        self.gain = 1.0
        self.has_limits = False
        self.upper_limit = 0.0
        self.lower_limit = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          len(self.level_name),
                          len(self.input),
                          len(self.output),
                          self.gain,
                          self.has_limits,
                          self.upper_limit,
                          self.lower_limit)
        msg += str.encode(self.level_name)
        msg += str.encode(self.input)
        msg += str.encode(self.output)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.level_name_len,
         self.input_len,
         self.output_len,
         self.gain,
         self.has_limits,
         self.upper_limit,
         self.lower_limit) = struct.unpack(self._pack_string, msg)
        self.level_name = extra[:self.level_name_len].decode()
        extra = extra[self.level_name_len:]
        self.input = extra[:self.input_len].decode()
        extra = extra[self.input_len:]
        self.output = extra[:self.output_len].decode()
        extra = extra[self.output_len:]

# Message: data_time
# Id: 40
class data_time():
    id = 40
    _pack_string = "<Q"

    def __init__(self, msg=None):
        # public fields
        self.time_us = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.time_us)
        return msg

    def unpack(self, msg):
        (self.time_us,) = struct.unpack(self._pack_string, msg)

# Message: data_mpu9250_short
# Id: 41
class data_mpu9250_short():
    id = 41
    _pack_string = "<bhhhhhh"

    def __init__(self, msg=None):
        # public fields
        self.ReadStatus = 0
        self.AccelX_mss = 0.0
        self.AccelY_mss = 0.0
        self.AccelZ_mss = 0.0
        self.GyroX_rads = 0.0
        self.GyroY_rads = 0.0
        self.GyroZ_rads = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.ReadStatus,
                          int(round(self.AccelX_mss * accel_scale)),
                          int(round(self.AccelY_mss * accel_scale)),
                          int(round(self.AccelZ_mss * accel_scale)),
                          int(round(self.GyroX_rads * gyro_scale)),
                          int(round(self.GyroY_rads * gyro_scale)),
                          int(round(self.GyroZ_rads * gyro_scale)))
        return msg

    def unpack(self, msg):
        (self.ReadStatus,
         self.AccelX_mss,
         self.AccelY_mss,
         self.AccelZ_mss,
         self.GyroX_rads,
         self.GyroY_rads,
         self.GyroZ_rads) = struct.unpack(self._pack_string, msg)
        self.AccelX_mss /= accel_scale
        self.AccelY_mss /= accel_scale
        self.AccelZ_mss /= accel_scale
        self.GyroX_rads /= gyro_scale
        self.GyroY_rads /= gyro_scale
        self.GyroZ_rads /= gyro_scale

# Message: data_mpu9250
# Id: 42
class data_mpu9250():
    id = 42
    _pack_string = "<bhhhhhhhhhh"

    def __init__(self, msg=None):
        # public fields
        self.ReadStatus = 0
        self.AccelX_mss = 0.0
        self.AccelY_mss = 0.0
        self.AccelZ_mss = 0.0
        self.GyroX_rads = 0.0
        self.GyroY_rads = 0.0
        self.GyroZ_rads = 0.0
        self.MagX_uT = 0.0
        self.MagY_uT = 0.0
        self.MagZ_uT = 0.0
        self.Temperature_C = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.ReadStatus,
                          int(round(self.AccelX_mss * accel_scale)),
                          int(round(self.AccelY_mss * accel_scale)),
                          int(round(self.AccelZ_mss * accel_scale)),
                          int(round(self.GyroX_rads * gyro_scale)),
                          int(round(self.GyroY_rads * gyro_scale)),
                          int(round(self.GyroZ_rads * gyro_scale)),
                          int(round(self.MagX_uT * mag_scale)),
                          int(round(self.MagY_uT * mag_scale)),
                          int(round(self.MagZ_uT * mag_scale)),
                          int(round(self.Temperature_C * 100)))
        return msg

    def unpack(self, msg):
        (self.ReadStatus,
         self.AccelX_mss,
         self.AccelY_mss,
         self.AccelZ_mss,
         self.GyroX_rads,
         self.GyroY_rads,
         self.GyroZ_rads,
         self.MagX_uT,
         self.MagY_uT,
         self.MagZ_uT,
         self.Temperature_C) = struct.unpack(self._pack_string, msg)
        self.AccelX_mss /= accel_scale
        self.AccelY_mss /= accel_scale
        self.AccelZ_mss /= accel_scale
        self.GyroX_rads /= gyro_scale
        self.GyroY_rads /= gyro_scale
        self.GyroZ_rads /= gyro_scale
        self.MagX_uT /= mag_scale
        self.MagY_uT /= mag_scale
        self.MagZ_uT /= mag_scale
        self.Temperature_C /= 100

# Message: data_bme280
# Id: 43
class data_bme280():
    id = 43
    _pack_string = "<bfhf"

    def __init__(self, msg=None):
        # public fields
        self.ReadStatus = 0
        self.Pressure_Pa = 0.0
        self.Temperature_C = 0.0
        self.Humidity_RH = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.ReadStatus,
                          self.Pressure_Pa,
                          int(round(self.Temperature_C * 100)),
                          self.Humidity_RH)
        return msg

    def unpack(self, msg):
        (self.ReadStatus,
         self.Pressure_Pa,
         self.Temperature_C,
         self.Humidity_RH) = struct.unpack(self._pack_string, msg)
        self.Temperature_C /= 100

# Message: data_ublox
# Id: 44
class data_ublox():
    id = 44
    _pack_string = "<BBLHBBBBBddfhhhfhhh"

    def __init__(self, msg=None):
        # public fields
        self.Fix = False
        self.NumberSatellites = 0
        self.TOW = 0
        self.Year = 0
        self.Month = 0
        self.Day = 0
        self.Hour = 0
        self.Min = 0
        self.Sec = 0
        self.Latitude_rad = 0.0
        self.Longitude_rad = 0.0
        self.Altitude_m = 0.0
        self.NorthVelocity_ms = 0.0
        self.EastVelocity_ms = 0.0
        self.DownVelocity_ms = 0.0
        self.HorizontalAccuracy_m = 0.0
        self.VerticalAccuracy_m = 0.0
        self.VelocityAccuracy_ms = 0.0
        self.pDOP = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.Fix,
                          self.NumberSatellites,
                          self.TOW,
                          self.Year,
                          self.Month,
                          self.Day,
                          self.Hour,
                          self.Min,
                          self.Sec,
                          self.Latitude_rad,
                          self.Longitude_rad,
                          self.Altitude_m,
                          int(round(self.NorthVelocity_ms * 100)),
                          int(round(self.EastVelocity_ms * 100)),
                          int(round(self.DownVelocity_ms * 100)),
                          self.HorizontalAccuracy_m,
                          int(round(self.VerticalAccuracy_m * 100)),
                          int(round(self.VelocityAccuracy_ms * 100)),
                          int(round(self.pDOP * 100)))
        return msg

    def unpack(self, msg):
        (self.Fix,
         self.NumberSatellites,
         self.TOW,
         self.Year,
         self.Month,
         self.Day,
         self.Hour,
         self.Min,
         self.Sec,
         self.Latitude_rad,
         self.Longitude_rad,
         self.Altitude_m,
         self.NorthVelocity_ms,
         self.EastVelocity_ms,
         self.DownVelocity_ms,
         self.HorizontalAccuracy_m,
         self.VerticalAccuracy_m,
         self.VelocityAccuracy_ms,
         self.pDOP) = struct.unpack(self._pack_string, msg)
        self.NorthVelocity_ms /= 100
        self.EastVelocity_ms /= 100
        self.DownVelocity_ms /= 100
        self.VerticalAccuracy_m /= 100
        self.VelocityAccuracy_ms /= 100
        self.pDOP /= 100

# Message: data_ams5915
# Id: 45
class data_ams5915():
    id = 45
    _pack_string = "<bfh"

    def __init__(self, msg=None):
        # public fields
        self.ReadStatus = 0
        self.Pressure_Pa = 0.0
        self.Temperature_C = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.ReadStatus,
                          self.Pressure_Pa,
                          int(round(self.Temperature_C * 100)))
        return msg

    def unpack(self, msg):
        (self.ReadStatus,
         self.Pressure_Pa,
         self.Temperature_C) = struct.unpack(self._pack_string, msg)
        self.Temperature_C /= 100

# Message: data_swift
# Id: 46
class data_swift():
    id = 46
    _pack_string = "<bfhbfh"

    def __init__(self, msg=None):
        # public fields
        self.static_ReadStatus = 0
        self.static_Pressure_Pa = 0.0
        self.static_Temperature_C = 0.0
        self.diff_ReadStatus = 0
        self.diff_Pressure_Pa = 0.0
        self.diff_Temperature_C = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.static_ReadStatus,
                          self.static_Pressure_Pa,
                          int(round(self.static_Temperature_C * 100)),
                          self.diff_ReadStatus,
                          self.diff_Pressure_Pa,
                          int(round(self.diff_Temperature_C * 100)))
        return msg

    def unpack(self, msg):
        (self.static_ReadStatus,
         self.static_Pressure_Pa,
         self.static_Temperature_C,
         self.diff_ReadStatus,
         self.diff_Pressure_Pa,
         self.diff_Temperature_C) = struct.unpack(self._pack_string, msg)
        self.static_Temperature_C /= 100
        self.diff_Temperature_C /= 100

# Message: data_sbus
# Id: 47
class data_sbus():
    id = 47
    _pack_string = "<hhhhhhhhhhhhhhhhBL"

    def __init__(self, msg=None):
        # public fields
        self.channels = [0.0] * 16
        self.FailSafe = False
        self.LostFrames = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          int(round(self.channels[0] * 20000)),
                          int(round(self.channels[1] * 20000)),
                          int(round(self.channels[2] * 20000)),
                          int(round(self.channels[3] * 20000)),
                          int(round(self.channels[4] * 20000)),
                          int(round(self.channels[5] * 20000)),
                          int(round(self.channels[6] * 20000)),
                          int(round(self.channels[7] * 20000)),
                          int(round(self.channels[8] * 20000)),
                          int(round(self.channels[9] * 20000)),
                          int(round(self.channels[10] * 20000)),
                          int(round(self.channels[11] * 20000)),
                          int(round(self.channels[12] * 20000)),
                          int(round(self.channels[13] * 20000)),
                          int(round(self.channels[14] * 20000)),
                          int(round(self.channels[15] * 20000)),
                          self.FailSafe,
                          self.LostFrames)
        return msg

    def unpack(self, msg):
        (self.channels[0],
         self.channels[1],
         self.channels[2],
         self.channels[3],
         self.channels[4],
         self.channels[5],
         self.channels[6],
         self.channels[7],
         self.channels[8],
         self.channels[9],
         self.channels[10],
         self.channels[11],
         self.channels[12],
         self.channels[13],
         self.channels[14],
         self.channels[15],
         self.FailSafe,
         self.LostFrames) = struct.unpack(self._pack_string, msg)
        self.channels[0] /= 20000
        self.channels[1] /= 20000
        self.channels[2] /= 20000
        self.channels[3] /= 20000
        self.channels[4] /= 20000
        self.channels[5] /= 20000
        self.channels[6] /= 20000
        self.channels[7] /= 20000
        self.channels[8] /= 20000
        self.channels[9] /= 20000
        self.channels[10] /= 20000
        self.channels[11] /= 20000
        self.channels[12] /= 20000
        self.channels[13] /= 20000
        self.channels[14] /= 20000
        self.channels[15] /= 20000

# Message: data_analog
# Id: 48
class data_analog():
    id = 48
    _pack_string = "<f"

    def __init__(self, msg=None):
        # public fields
        self.calibrated_value = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.calibrated_value)
        return msg

    def unpack(self, msg):
        (self.calibrated_value,) = struct.unpack(self._pack_string, msg)

# Message: data_compound
# Id: 49
class data_compound():
    id = 49
    _pack_string = "<"

    def __init__(self, msg=None):
        # public fields
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
        return msg

    def unpack(self, msg):

