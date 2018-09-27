/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2018 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MESSAGES_H
#define MESSAGES_H

#include "checksum.h"
#include "array_macros.h"
#include "global_defs.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
#define _Bool bool
extern "C" {
#endif

/* 
* Defines message content for Node <--> FMU <--> SOC communication.
* Configuration messages are unique for each device (i.e. AMS-5915)
* while data messages are common for each type (i.e. static pressure).
* All units are SI units where appropriate.
*/

enum MessageIds {
  MSG_FMU_CFG,
  MSG_INT_MPU9250_CFG,
  MSG_INT_BME280_CFG,
  MSG_MPU9250_CFG,
  MSG_BME280_CFG,
  MSG_UBLOX_CFG,
  MSG_SBUS_CFG,
  MSG_SWIFT_CFG,
  MSG_AMS5915_CFG,
  MSG_ANALOG_CFG,
  MSG_DIGITAL_CFG,
  MSG_VOLTAGE_CFG,
  MSG_TIME_DATA,
  MSG_ACCEL_DATA,
  MSG_GYRO_DATA,
  MSG_MAG_DATA,
  MSG_INCEPTOR_DATA,
  MSG_TEMPERATURE_DATA,
  MSG_GNSS_DATA,
  MSG_STATIC_PRESS_DATA,
  MSG_DIFF_PRESS_DATA,
  MSG_ANALOG_DATA,
  MSG_DIGITAL_DATA,
  MSG_VOLTAGE_DATA,
  NUM_MSG_ID
};
typedef enum MessageIds MsgId_t;

unsigned char *msg_build_i2c(unsigned int hash,MsgId_t msg,void *payload);
void *msg_parse_i2c(unsigned char *buffer,unsigned int *hash,MsgId_t *msg);
unsigned char *msg_build_serial(unsigned int hash,MsgId_t msg,void *payload);
void *msg_parse_serial(unsigned char byte,unsigned int *hash,MsgId_t *msg);

#pragma pack(push,1)

/* 
* FMU configuration message. Configures the sample rate divider to set
* the flight computer frame rate, where the frequency is 1000 / (1 + srd).
* Also configures the FMU orientation relative to the aircraft as a yaw,
* roll, pitch sequence of rotations. FMU positive x axis is out the FMU
* USB port, y is to the right, and z is down.
*/
struct msg_fmu_config {
  unsigned int srd : 8;                   // sample rate divider
  unsigned int rot_yaw_rad : 10;          // yaw rotation, +/- pi range
  unsigned int rot_roll_rad : 10;         // roll rotation, +/- pi range
  unsigned int rot_pitch_rad : 9;         // pitch rotation, +/- pi/2 range
};
/*
* FMU integrated MPU-9250 configuration. Communication bus (SPI) and CS
* pin are fixed values. Orientation is fixed and configurable by the FMU
* orientation. SRD is configurable by the FMU SRD. Configurable accel
* and gyro full scale ranges, bandwidth, accel bias, mag bias, and mag
* scale factor.
*/
struct msg_int_mpu9250_config {
  unsigned int accel_range : 2;           // accelerometer range (MPU9250::AccelRange)
  unsigned int gyro_range : 2;            // gyro range (MPU9250::GyroRange)
  unsigned int bandwidth : 3;             // bandwidth (MPU9250::DlpfBandwidth)
  unsigned int ax_bias_mss : 11;          // ax bias (+/- 1 m/s), 0.001 m/s resolution
  unsigned int ay_bias_mss : 11;          // ay bias (+/- 1 m/s)
  unsigned int az_bias_mss : 11;          // az bias (+/- 1 m/s)
  unsigned int hx_bias_ut : 10;           // hx bias (+/- 300 uT), 0.6 uT resolution
  unsigned int hy_bias_ut : 10;           // hy bias (+/- 300 uT)
  unsigned int hz_bias_ut : 10;           // hz bias (+/- 300 uT)
  unsigned int hx_scale : 10;             // hx scale (0 - 2), 0.2% resolution
  unsigned int hy_scale : 10;             // hy scale (0 - 2)
  unsigned int hz_scale : 10;             // hz scale (0 - 2)
};
/*
* FMU integrated BME-280 configuration. Communication bus (SPI) and CS
* pin are fixed values. Currently no configurable items.
*/
struct msg_int_bme280_config {};
/*
* External MPU-9250 configuration. Specify the address of the FMU or Node the
* sensor is connected to, whether it's communicating over SPI (only available
* on the FMU with a fixed CS pin) or I2C, the I2C bus (only one bus available
* on the FMU with two available on the Node), the I2C address (two available),
* accel range, gyro range, sample rate divider, bandwidth, orientation as a
* yaw, roll, pitch sequence of rotations, accel bias, mag bias, and mag
* scale factor.
*/
struct msg_mpu9250_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
  unsigned int use_spi : 1;               // 0 = i2c, 1 = spi
  unsigned int bus : 1;                   // i2c bus (0 = bus 1, 1 = bus 2)
  unsigned int addr : 1;                  // i2c addr (0 = 0x68, 1 = 0x69)
  unsigned int accel_range : 2;           // accelerometer range (MPU9250::AccelRange)
  unsigned int gyro_range : 2;            // gyro range (MPU9250::GyroRange)
  unsigned int srd : 8;                   // sample rate divider
  unsigned int bandwidth : 3;             // bandwidth (MPU9250::DlpfBandwidth)
  unsigned int rot_yaw : 10;              // yaw rotation, +/- pi, 0.5 degree resolution
  unsigned int rot_roll : 10;             // roll rotation, +/- pi, 0.5 degree resolution
  unsigned int rot_pitch : 9;             // pitch rotation, +/- pi/2, 0.5 degree resolution
  unsigned int ax_bias_mss : 11;          // ax bias (+/- 1 m/s), 0.001 m/s resolution
  unsigned int ay_bias_mss : 11;          // ay bias (+/- 1 m/s)
  unsigned int az_bias_mss : 11;          // az bias (+/- 1 m/s)
  unsigned int hx_bias_ut : 10;           // hx bias (+/- 300 uT), 0.6 uT resolution
  unsigned int hy_bias_ut : 10;           // hy bias (+/- 300 uT)
  unsigned int hz_bias_ut : 10;           // hz bias (+/- 300 uT)
  unsigned int hx_scale : 10;             // hx scale (0 - 2), 0.2% resolution
  unsigned int hy_scale : 10;             // hy scale (0 - 2)
  unsigned int hz_scale : 10;             // hz scale (0 - 2)
};
/*
* External BME-280 configuration. Specify the address of the FMU or Node the
* sensor is connected to, whether it's communicating over SPI (only available
* on the FMU with a fixed CS pin) or I2C, the I2C bus (only one bus available
* on the FMU with two available on the Node), and the I2C address 
* (two available).
*/
struct msg_bme280_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
  unsigned int use_spi : 1;               // 0 = i2c, 1 = spi
  unsigned int bus : 1;                   // i2c bus (0 = bus 1, 1 = bus 2)
  unsigned int addr : 1;                  // i2c addr (0 = 0x76, 1 = 0x77)
};
/*
* uBlox GNSS configuration. Specify the address of the FMU or Node the
* sensor is connected to, the UART port (ports 1 and 2 available on Node, 
* ports 3 and 4 available on FMU), and the baud rate (see listed rates).
*/
struct msg_ublox_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
  unsigned int uart : 2;                  // uart port (0 - 4)
  unsigned int baud : 3;                  // baud rate (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)
};
/*
* SBUS receiver configuration, specify the address of the FMU or Node
* the receiver is connected to. The FMU and Nodes have dedicated SBUS
* receiver ports, so the port does not need to be specified.
*/
struct msg_sbus_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
};
/*
* Swift air data configuration. The Swift consists of a static and 
* differential AMS-5915 pressure transducer. Three differential
* transducers are available by request for the Swift depending on 
* the speed range needed. Specify the address of the FMU or Node
* the Swift is connected to, the I2C bus (only one bus available
* on the FMU with two available on the Node), the static and
* differential I2C addresses, and the differential transducer
* type (see the listed types).
*/
struct msg_swift_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
  unsigned int bus : 1;                   // i2c bus (0 = bus 1, 1 = bus 2)
  unsigned int static_addr: 7;            // static sensor 12c address
  unsigned int diff_addr: 7;              // diff sensor i2c address
  unsigned int diff_type: 2;              // diff transducer type (5915-0005, 5915-0010, 5915-0020)
};
/*
* AMS-5915 pressure transduce configuration. Specify the address
* of the FMU or Node the sensor is connected to, the I2C bus 
* (only one bus available on the FMU with two available on the Node),
* the I2C address, and the transducer type (see the listed types).
*/
struct msg_ams5915_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
  unsigned int bus : 1;                   // i2c bus (0 = bus 1, 1 = bus 2)
  unsigned int addr: 7;                   // i2c address
  unsigned int type: 5;                   // transducer type (AMS5915::Transducer)
};
/*
* Analog sensor configuration. Data output is given as both a voltage value
* and a calibrated output (i.e. using a POT to measure control surface
* position). Specify the address of the FMU or Node for the analog input 
* (2 available on the FMU, 8 available on the Node), the channel number for 
* the analog input, the number of polynomial coefficients used in calibiration
* and an array of those coefficients given in descending order.
*/
struct msg_analog_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
  unsigned int ch : 3;                    // analog channel number
  unsigned int coeff_len : 3;             // number of polynomial coefficients used
  unsigned int coeff0 : 32;                      // polynomial coefficients
  unsigned int coeff1 : 32;                      // polynomial coefficients
  unsigned int coeff2 : 32;                      // polynomial coefficients
  unsigned int coeff3 : 32;                      // polynomial coefficients
  unsigned int coeff4 : 32;                      // polynomial coefficients
  unsigned int coeff5 : 32;                      // polynomial coefficients
  unsigned int coeff6 : 32;                      // polynomial coefficients
  unsigned int coeff7 : 32;                      // polynomial coefficients
};
/*
* Digital sensor configuration. Data output is given as a boolean value.
* Specify the address of the FMU or Node for the digital input (2 available
* on the FMU, 4 available on the Node), the channel number, and whether a 
* high digital reading should result in a true or false data output.
*/
struct msg_digital_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
  unsigned int ch : 2;                    // digital channel number (2 GPIO on FMU, 4 digital pins on Node)
  unsigned int active_high : 1;           // 0 = read high, output 0, 1 = read high, output 1
};
/* 
* Voltage sensor configuration. On the FMU available data includes: input
* voltage (6.5 - 36V), regulated voltage (nominally ~5V), PWM servo voltage
* (0 - 9.9V), and SBUS servo voltage (0 - 9.9V). On the Node, PWM and SBUS
* servo voltages are available. Specify the  FMU or Node address for the
* voltage sensor, and the type (see the listed types).
*/
struct msg_voltage_config {
  unsigned int bfs_addr : 3;              // bfs addr of component (0 - 7)
  unsigned int type : 2;                  // voltage measurement (0 = input, 1 = regulated, 2 = pwm, 3 = sbus)
};
/* 
* Time data, includes a rolling frame counter to sync data and a counter
* of the total number of frames. Coupled with the SRD, this can be used
* to generate a time value.
*/
struct msg_time_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int frames : 25;               // counts the total number of frames
};
/* 
* Accelerometer data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Also includes three axis accelerometer values with a range
* of +/- 16G or +/- 157 m/s/s.
*/
struct msg_accel_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int accel_x_mss : 19;          // accel, +/-16G range or +/- 157 m/s/s
  unsigned int accel_y_mss : 19;
  unsigned int accel_z_mss : 19;
};
/* 
* Gyro data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Also includes three axis gyro values with a range
* of +/- 2000 deg/s or 35 rad/s/
*/
struct msg_gyro_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int gyro_x_rads : 22;          // gyro, +/- 2000 deg/s range or +/- 35 rad/s
  unsigned int gyro_y_rads : 22;
  unsigned int gyro_z_rads : 22;
};
/* 
* Magnetometer data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Also includes three axis mag values with a range
* of +/- 1000 uT.
*/
struct msg_mag_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int mag_x_ut : 18;             // mag, +/- 1000 uT range
  unsigned int mag_y_ut : 18;
  unsigned int mag_z_ut : 18;
};
/* 
* Inceptor data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Also includes boolean values for whether a frame was lost,
* whether the receiver is in failsafe mode, and 16 channels of normalized
* pilot input data. 
*/
struct msg_inceptor_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int lost_frame : 1;            // 0 = frame received, 1 = frame lost
  unsigned int failsafe_activated : 1;    // 0 = failsafe inactive, 1 = failsafe active
  unsigned int ch0 : 11;                  // SBUS channel data, +/- 1 range
  unsigned int ch1 : 11;                  
  unsigned int ch2 : 11;                  
  unsigned int ch3 : 11;                  
  unsigned int ch4 : 11;                  
  unsigned int ch5 : 11;                  
  unsigned int ch6 : 11;                  
  unsigned int ch7 : 11;                  
  unsigned int ch8 : 11;                  
  unsigned int ch9 : 11;                  
  unsigned int ch10 : 11;                 
  unsigned int ch11 : 11;                 
  unsigned int ch12 : 11;                 
  unsigned int ch13 : 11;                 
  unsigned int ch14 : 11;                 
  unsigned int ch15 : 11;                   
};
/* 
* Temperature data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Temperature data is valid in the range -40 to +80 C.
*/
struct msg_temperature_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int temp_c : 14;               // temperature, C, -40 to +80 C
};
/* 
* GNSS data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. GNSS data includes a boolean value of whether a good fix has
* been acheived, the number of satellites being tracked, the GPS time of week,
* the latitude, longitude, altitude, NED velocity, estimates of horizontal, 
* vertical and speed accuracy, and the position dilution of precision.
*/
struct msg_gnss_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int fix : 1;                   // fix (0 = no fix, 1 = good fix)
  unsigned int num_sv : 5;                // number of satellites tracked
  unsigned int tow_ms : 32;               // GPS TOW, ms
  unsigned int lat_rad : 32;              // latitude, rad
  unsigned int lon_rad : 32;              // longitude, rad
  unsigned int hmsl_m : 26;               // MSL altitude, m
  unsigned int vel_north_ms : 20;         // NED north velocity, m/s
  unsigned int vel_east_ms : 20;          // NED east velocity, m/s
  unsigned int vel_down_ms : 18;          // NED down velocity, m/s
  unsigned int horiz_acc_m : 17;          // horizontal accuracy estimate, m
  unsigned int vert_acc_m : 17;           // vertical accuracy estimate, m
  unsigned int speed_acc_ms : 17;         // speed accuracy estimate, m/s
  unsigned int pdop : 14;                 // position dilution of precision
};
/* 
* Static pressure data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Pressure data is valid in the range from 30000 to 110000 Pa.
*/
struct msg_static_press_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int press_pa : 22;             // pressure, pa 30000 to 110000 Pa
};
/* 
* Differential pressure data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Pressure data is valid in the range +/- 20000 Pa.
*/
struct msg_diff_press_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int press_pa : 22;
};
/* 
* Analog data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Voltage given in the range of 0 - 3.3V as well as a
* calibrated value from evaluating polynomial coefficients.
*/
struct msg_analog_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int voltage : 13;              // measured voltage
  unsigned int cal_value : 32;            // calibrated value
};
/* 
* Digital data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Digital output is given as a boolean.
*/
struct msg_digital_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int digout : 1;                // high or low
};
/* 
* Voltage data, includes a rolling frame counter to sync data and
* a time offset from the start of the frame in us for when the data was
* collected. Voltage data is valid from 0 - 36V.
*/
struct msg_voltage_data {
  unsigned int frame_counter : 3;         // a rolling frame counter to sync data packets
  unsigned int time_offset_us : 15;       // time offset from the frame start time, us
  unsigned int voltage : 13;              // measured voltage
};

#pragma pack(pop)

/*
* Functions to convert the unpacked structures to the packed structures and back
*/
void pack_fmu_config(void *unpacked, void *packed);
void unpack_fmu_config(struct msg_fmu_config *packed, struct fmu_config *unpacked);
void pack_int_mpu9250_config(struct int_mpu9250_config *unpacked, struct msg_int_mpu9250_config *packed);
void unpack_int_mpu9250_config(struct msg_int_mpu9250_config *packed, struct int_mpu9250_config *unpacked);
void pack_int_bme280_config(struct int_bme280_config *unpacked, struct msg_int_bme280_config *packed);
void unpack_int_bme280_config(struct msg_int_bme280_config *packed, struct int_bme280_config *unpacked);
void pack_mpu9250_config(struct mpu9250_config *unpacked, struct msg_mpu9250_config *packed);
void unpack_mpu9250_config(struct msg_mpu9250_config *packed, struct mpu9250_config *unpacked);
void pack_bme280_config(struct bme280_config *unpacked, struct msg_bme280_config *packed);
void unpack_bme280_config(struct msg_bme280_config *packed, struct bme280_config *unpacked);
void pack_ublox_config(struct ublox_config *unpacked, struct msg_ublox_config *packed);
void unpack_ublox_config(struct msg_ublox_config *packed, struct ublox_config *unpacked);
void pack_sbus_config(struct sbus_config *unpacked, struct msg_sbus_config *packed);
void unpack_sbus_config(struct msg_sbus_config *packed, struct sbus_config *unpacked);
void pack_swift_config(struct swift_config *unpacked, struct msg_swift_config *packed);
void unpack_swift_config(struct msg_swift_config *packed, struct swift_config *unpacked);
void pack_ams5915_config(struct ams5915_config *unpacked, struct msg_ams5915_config *packed);
void unpack_ams5915_config(struct msg_ams5915_config *packed, struct ams5915_config *unpacked);
void pack_analog_config(struct analog_config *unpacked, struct msg_analog_config *packed);
void unpack_analog_config(struct msg_analog_config *packed, struct analog_config *unpacked);
void pack_digital_config(struct digital_config *unpacked, struct msg_digital_config *packed);
void unpack_digital_config(struct msg_digital_config *packed, struct digital_config *unpacked);
void pack_voltage_config(struct voltage_config *unpacked, struct msg_voltage_config *packed);
void unpack_voltage_config(struct msg_voltage_config *packed, struct voltage_config *unpacked);
void pack_time_data(struct time_data *unpacked, struct msg_time_data *packed);
void unpack_time_data(struct msg_time_data *packed, struct time_data *unpacked);
void pack_accel_data(struct accel_data *unpacked, struct msg_accel_data *packed);
void unpack_accel_data(struct msg_accel_data *packed, struct accel_data *unpacked);
void pack_gyro_data(struct gyro_data *unpacked, struct msg_gyro_data *packed);
void unpack_gyro_data(struct msg_gyro_data *packed, struct gyro_data *unpacked);
void pack_mag_data(struct mag_data *unpacked, struct msg_mag_data *packed);
void unpack_mag_data(struct msg_mag_data *packed, struct mag_data *unpacked);
void pack_inceptor_data(struct inceptor_data *unpacked, struct msg_inceptor_data *packed);
void unpack_inceptor_data(struct msg_inceptor_data *packed, struct inceptor_data *unpacked);
void pack_temperature_data(struct temperature_data *unpacked, struct msg_temperature_data *packed);
void unpack_temperature_data(struct msg_temperature_data *packed, struct temperature_data *unpacked);
void pack_gnss_data(struct gnss_data *unpacked, struct msg_gnss_data *packed);
void unpack_gnss_data(struct msg_gnss_data *packed, struct gnss_data *unpacked);
void pack_static_press_data(struct static_press_data *unpacked, struct msg_static_press_data *packed);
void unpack_static_press_data(struct msg_static_press_data *packed, struct static_press_data *unpacked);
void pack_diff_press_data(struct diff_press_data *unpacked, struct msg_diff_press_data *packed);
void unpack_diff_press_data(struct msg_diff_press_data *packed, struct diff_press_data *unpacked);
void pack_analog_data(struct analog_data *unpacked, struct msg_analog_data *packed);
void unpack_analog_data(struct msg_analog_data *packed, struct analog_data *unpacked);
void pack_digital_data(struct digital_data *unpacked, struct msg_digital_data *packed);
void unpack_digital_data(struct msg_digital_data *packed, struct digital_data *unpacked);
void pack_voltage_data(struct voltage_data *unpacked, struct msg_voltage_data *packed);
void unpack_voltage_data(struct msg_voltage_data *packed, struct voltage_data *unpacked);

/*
* Ensure C friendly linkages in a mixed C/C++ build
*/
#ifdef __cplusplus
}
#endif

#endif
