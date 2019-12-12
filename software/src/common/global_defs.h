/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#ifndef GLOBAL_DEFS_H
#define GLOBAL_DEFS_H

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
#define _Bool bool
extern "C" {
#endif

/* standard gravity */
#define G 9.80665

/* degrees to rads */
#define D2R M_PI / 180.0f

/* radians to degrees */
#define R2D 180.0f / M_PI

/* double precision PI */
#define D_PI 3.141592653589793

/*
* FMU configuration. Configures the sample rate divider to set
* the flight computer frame rate, where the frequency is 1000 / (1 + srd).
* Also configures the FMU orientation relative to the aircraft as a yaw,
* roll, pitch sequence of rotations. FMU positive x axis is out the FMU
* USB port, y is to the right, and z is down.
*/
struct fmu_config {
  unsigned int srd;                       // sample rate divider
  float rot_yaw_rad;                      // yaw rotation
  float rot_roll_rad;                     // roll rotation
  float rot_pitch_rad;                    // pitch rotation
};
/*
* FMU integrated MPU-9250 configuration. Communication bus (SPI) and CS
* pin are fixed values. Orientation is fixed and configurable by the FMU
* orientation. SRD is configurable by the FMU SRD. Configurable accel
* and gyro full scale ranges, bandwidth, accel bias, mag bias, and mag
* scale factor.
*/
struct int_mpu9250_config {
  unsigned int accel_range;               // accelerometer range (MPU9250::AccelRange)
  unsigned int gyro_range;                // gyro range (MPU9250::GyroRange)
  unsigned int bandwidth;                 // bandwidth (MPU9250::DlpfBandwidth)
  float ax_bias_mss;                      // ax bias
  float ay_bias_mss;                      // ay bias
  float az_bias_mss;                      // az bias
  float hx_bias_ut;                       // hx bias
  float hy_bias_ut;                       // hy bias
  float hz_bias_ut;                       // hz bias
  float hx_scale;                         // hx scale
  float hy_scale;                         // hy scale
  float hz_scale;                         // hz scale
};
/*
* FMU integrated BME-280 configuration. Communication bus (SPI) and CS
* pin are fixed values. Currently no configurable items.
*/
struct int_bme280_config {};
/*
* External MPU-9250 configuration. Specify the address of the FMU or Node the
* sensor is connected to, whether it's communicating over SPI (only available
* on the FMU with a fixed CS pin) or I2C, the I2C bus (only one bus available
* on the FMU with two available on the Node), the I2C address (two available),
* accel range, gyro range, sample rate divider, bandwidth, orientation as a
* yaw, roll, pitch sequence of rotations, accel bias, mag bias, and mag
* scale factor.
*/
struct mpu9250_config {
  unsigned int bfs_addr;                  // bfs addr of component
  _Bool use_spi;                          // 0 = i2c, 1 = spi
  unsigned int bus;                       // i2c bus
  unsigned int addr;                      // i2c addr
  unsigned int accel_range;               // accelerometer range (MPU9250::AccelRange)
  unsigned int gyro_range;                // gyro range (MPU9250::GyroRange)
  unsigned int srd;                       // sample rate divider
  unsigned int bandwidth;                 // bandwidth (MPU9250::DlpfBandwidth)
  float rot_yaw;                          // yaw rotation,
  float rot_roll;                         // roll rotation,
  float rot_pitch;                        // pitch rotation,
  float ax_bias_mss;                      // ax bias
  float ay_bias_mss;                      // ay bias
  float az_bias_mss;                      // az bias
  float hx_bias_ut;                       // hx bias
  float hy_bias_ut;                       // hy bias
  float hz_bias_ut;                       // hz bias
  float hx_scale;                         // hx scale
  float hy_scale;                         // hy scale
  float hz_scale;                         // hz scale
};
/*
* External BME-280 configuration. Specify the address of the FMU or Node the
* sensor is connected to, whether it's communicating over SPI (only available
* on the FMU with a fixed CS pin) or I2C, the I2C bus (only one bus available
* on the FMU with two available on the Node), and the I2C address
* (two available).
*/
struct bme280_config {
  unsigned int bfs_addr;                  // bfs addr of component
  _Bool use_spi;                          // 0 = i2c, 1 = spi
  unsigned int bus;                       // i2c bus
  unsigned int addr;                      // i2c addr
};
/*
* uBlox GNSS configuration. Specify the address of the FMU or Node the
* sensor is connected to, the UART port (ports 1 and 2 available on Node,
* ports 3 and 4 available on FMU), and the baud rate.
*/
struct ublox_config {
  unsigned int bfs_addr;                  // bfs addr of component
  unsigned int uart;                      // uart port
  unsigned long baud;                     // baud rate
};
/*
* SBUS receiver configuration, specify the address of the FMU or Node
* the receiver is connected to. The FMU and Nodes have dedicated SBUS
* receiver ports, so the port does not need to be specified.
*/
struct sbus_config {
  unsigned int bfs_addr;                  // bfs addr of component
};
/*
* Swift air data configuration. The Swift consists of a static and
* differential AMS-5915 pressure transducer. Three differential
* transducers are available by request for the Swift depending on
* the speed range needed. Specify the address of the FMU or Node
* the Swift is connected to, the I2C bus (only one bus available
* on the FMU with two available on the Node), the static and
* differential I2C addresses, and the differential transducer
* type.
*/
struct swift_config {
  unsigned int bfs_addr;                  // bfs addr of component
  unsigned int bus;                       // i2c bus
  unsigned int static_addr;               // static sensor 12c address
  unsigned int diff_addr;                 // diff sensor i2c address
  unsigned int diff_type;                 // diff transducer type (AMS5915::Transducer)
};
/*
* AMS-5915 pressure transduce configuration. Specify the address
* of the FMU or Node the sensor is connected to, the I2C bus
* (only one bus available on the FMU with two available on the Node),
* the I2C address, and the transducer type.
*/
struct ams5915_config {
  unsigned int bfs_addr;                  // bfs addr of component
  unsigned int bus ;                      // i2c bus
  unsigned int addr;                      // i2c address
  unsigned int type;                      // transducer type (AMS5915::Transducer)
};
/*
* Analog sensor configuration. Data output is given as both a voltage value
* and a calibrated output (i.e. using a POT to measure control surface
* position). Specify the address of the FMU or Node for the analog input
* (2 available on the FMU, 8 available on the Node), the channel number for
* the analog input, the number of polynomial coefficients used in calibiration
* and an array of those coefficients given in descending order.
*/
struct analog_config {
  unsigned int bfs_addr;                  // bfs addr of component
  unsigned int ch;                        // analog channel number
  unsigned int coeff_len;                 // number of polynomial coefficients used
  float coeff[8];                         // polynomial coefficients
};
/*
* Digital sensor configuration. Data output is given as a boolean value.
* Specify the address of the FMU or Node for the digital input (2 available
* on the FMU, 4 available on the Node), the channel number, and whether a
* high digital reading should result in a true or false data output.
*/
struct digital_config {
  unsigned int bfs_addr;                  // bfs addr of component
  unsigned int ch;                        // digital channel number (2 GPIO on FMU, 4 digital pins on Node)
  unsigned int active_high;               // 0 = read high, output 0, 1 = read high, output 1
};
/*
* Voltage sensor configuration. On the FMU available data includes: input
* voltage (6.5 - 36V), regulated voltage (nominally ~5V), PWM servo voltage
* (0 - 9.9V), and SBUS servo voltage (0 - 9.9V). On the Node, PWM and SBUS
* servo voltages are available. Specify the  FMU or Node address for the
* voltage sensor, and the type (see the listed types).
*/
struct voltage_config {
  unsigned int bfs_addr;                  // bfs addr of component
  unsigned int type;                      // voltage measurement (0 = input, 1 = regulated, 2 = pwm, 3 = sbus)
};
/*
* Time data, includes a rolling frame counter to sync data and a counter
* of the total number of frames. Coupled with the SRD, this can be used
* to generate a time value.
*/
struct time_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned long frames;                   // counts the total number of frames
};
/*
* Accelerometer data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Also includes three axis accelerometer values.
*/
struct accel_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  float accel_x_mss;                      // accel
  float accel_y_mss;
  float accel_z_mss;
};
/*
* Gyro data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Also includes three axis gyro values.
*/
struct gyro_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  float gyro_x_rads;                      // gyro
  float gyro_y_rads;
  float gyro_z_rads;
};
/*
* Magnetometer data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Also includes three axis mag values.
*/
struct mag_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  float mag_x_ut;                         // mag
  float mag_y_ut;
  float mag_z_ut;
};
/*
* Inceptor data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Also includes boolean values for whether a frame was lost,
* whether the receiver is in failsafe mode, and 16 channels of normalized
* pilot input data.
*/
struct inceptor_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  _Bool lost_frame;                       // 0 = frame received, 1 = frame lost
  _Bool failsafe_activated;               // 0 = failsafe inactive, 1 = failsafe active
  float ch[16];                           // SBUS channel data
};
/*
* Temperature data, includes a rolling frame counter to sync data,
* a time offset from the start of the frame in us for when the data was
* collected, and temperature data.
*/
struct temperature_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  float temp_c;                           // temperature, C
};
/*
* GNSS data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. GNSS data includes a boolean value of whether a good fix has
* been acheived, the number of satellites being tracked, the GPS time of week,
* the latitude, longitude, altitude, NED velocity, estimates of horizontal,
* vertical and speed accuracy, and the position dilution of precision.
*/
struct gnss_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  _Bool fix;                              // fix (0 = no fix, 1 = good fix)
  unsigned int num_sv;                    // number of satellites tracked
  unsigned long tow_ms;                   // GPS TOW, ms
  double lat_rad;                         // latitude, rad
  double lon_rad;                         // longitude, rad
  float hmsl_m;                           // MSL altitude, m
  float vel_north_ms;                     // NED north velocity, m/s
  float vel_east_ms;                      // NED east velocity, m/s
  float vel_down_ms;                      // NED down velocity, m/s
  float horiz_acc_m;                      // horizontal accuracy estimate, m
  float vert_acc_m;                       // vertical accuracy estimate, m
  float speed_acc_ms;                     // speed accuracy estimate, m/s
  float pdop;                             // position dilution of precision
};
/*
* Static pressure data, includes a rolling frame counter to sync data,
* a time offset from the start of the frame in us for when the data was
* collected, and pressure data.
*/
struct static_press_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  float press_pa;                         // pressure, pa
};
/*
* Differential pressure data, includes a rolling frame counter to sync data,
* a time offset from the start of the frame in us for when the data was
* collected, and pressure data.
*/
struct diff_press_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  float press_pa;
};
/*
* Analog data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Voltage given in the range of 0 - 3.3V as well as a
* calibrated value from evaluating polynomial coefficients.
*/
struct analog_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  float voltage;                          // measured voltage
  float cal_value;                        // calibrated value
};
/*
* Digital data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Digital output is given as a boolean.
*/
struct digital_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  _Bool digout;                           // high or low
};
/*
* Voltage data, includes a rolling frame counter to sync data,
* a time offset from the start of the frame in us for when the data was
* collected, and voltage data.
*/
struct voltage_data {
  unsigned int frame_counter;             // a rolling frame counter to sync data packets
  unsigned int time_offset_us;            // time offset from the frame start time, us
  float voltage;                          // measured voltage
};

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
}
#endif

#endif
