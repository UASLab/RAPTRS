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

#include "messages.h"

/*
* Message header and footer sizes.
*/
#define MSG_HEADER_LEN 8
#define MSG_CHECKSUM_LEN 2

/*
* Message header.
*/
static const unsigned char MSG_HEADER[2] = {0x42,0x46};

/*
* Max length of packed and unpacked messages. Must have run msg_init
* for these values to be valid.
*/
unsigned int msg_max_packed_len, msg_max_unpacked_len;
/*
* Packed message length, looked up by message id. Must have run msg_init
* for these values to be valid.
*/
unsigned int msg_packed_len[NUM_MSG_ID];
/*
* Unpacked message length, looked up by message id. Must have run msg_init
* for these values to be valid.
*/
unsigned int msg_unpacked_len[NUM_MSG_ID];
/*
* Array of function pointers to pack a struct, looked up by message id.
*/
void (*pack_msg[NUM_MSG_ID]) (void *unpacked,void *packed);
/*
* Array of function pointers to unpack a struct, looked up by message id.
*/
void (*unpack_msg[NUM_MSG_ID]) (void *packed,void *unpacked);

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
void unpack_fmu_config(void *packed, void *unpacked);
void pack_int_mpu9250_config(void *unpacked, void *packed);
void unpack_int_mpu9250_config(void *packed, void *unpacked);
void pack_int_bme280_config(void *unpacked, void *packed);
void unpack_int_bme280_config(void *packed, void *unpacked);
void pack_mpu9250_config(void *unpacked, void *packed);
void unpack_mpu9250_config(void *packed, void *unpacked);
void pack_bme280_config(void *unpacked, void *packed);
void unpack_bme280_config(void *packed, void *unpacked);
void pack_ublox_config(void *unpacked, void *packed);
void unpack_ublox_config(void *packed, void *unpacked);
void pack_sbus_config(void *unpacked, void *packed);
void unpack_sbus_config(void *packed, void *unpacked);
void pack_swift_config(void *unpacked, void *packed);
void unpack_swift_config(void *packed, void *unpacked);
void pack_ams5915_config(void *unpacked, void *packed);
void unpack_ams5915_config(void *packed, void *unpacked);
void pack_analog_config(void *unpacked, void *packed);
void unpack_analog_config(void *packed, void *unpacked);
void pack_digital_config(void *unpacked, void *packed);
void unpack_digital_config(void *packed, void *unpacked);
void pack_voltage_config(void *unpacked, void *packed);
void unpack_voltage_config(void *packed, void *unpacked);
void pack_time_data(void *unpacked, void *packed);
void unpack_time_data(void *packed, void *unpacked);
void pack_accel_data(void *unpacked, void *packed);
void unpack_accel_data(void *packed, void *unpacked);
void pack_gyro_data(void *unpacked, void *packed);
void unpack_gyro_data(void *packed, void *unpacked);
void pack_mag_data(void *unpacked, void *packed);
void unpack_mag_data(void *packed, void *unpacked);
void pack_inceptor_data(void *unpacked, void *packed);
void unpack_inceptor_data(void *packed, void *unpacked);
void pack_temperature_data(void *unpacked, void *packed);
void unpack_temperature_data(void *packed, void *unpacked);
void pack_gnss_data(void *unpacked, void *packed);
void unpack_gnss_data(void *packed, void *unpacked);
void pack_static_press_data(void *unpacked, void *packed);
void unpack_static_press_data(void *packed, void *unpacked);
void pack_diff_press_data(void *unpacked, void *packed);
void unpack_diff_press_data(void *packed, void *unpacked);
void pack_analog_data(void *unpacked, void *packed);
void unpack_analog_data(void *packed, void *unpacked);
void pack_digital_data(void *unpacked, void *packed);
void unpack_digital_data(void *packed, void *unpacked);
void pack_voltage_data(void *unpacked, void *packed);
void unpack_voltage_data(void *packed, void *unpacked);

Msg_t *msg_init()
{
  Msg_t *msg;
  unsigned int i;
  /* array of packed message lengths */
  msg_packed_len[MSG_FMU_CFG]           = sizeof(struct msg_fmu_config);
  msg_packed_len[MSG_INT_MPU9250_CFG]   = sizeof(struct msg_int_mpu9250_config);
  msg_packed_len[MSG_INT_BME280_CFG]    = sizeof(struct msg_int_bme280_config);
  msg_packed_len[MSG_MPU9250_CFG]       = sizeof(struct msg_mpu9250_config);
  msg_packed_len[MSG_BME280_CFG]        = sizeof(struct msg_bme280_config);
  msg_packed_len[MSG_UBLOX_CFG]         = sizeof(struct msg_ublox_config);
  msg_packed_len[MSG_SBUS_CFG]          = sizeof(struct msg_sbus_config);
  msg_packed_len[MSG_SWIFT_CFG]         = sizeof(struct msg_swift_config);
  msg_packed_len[MSG_AMS5915_CFG]       = sizeof(struct msg_ams5915_config);
  msg_packed_len[MSG_ANALOG_CFG]        = sizeof(struct msg_analog_config);
  msg_packed_len[MSG_DIGITAL_CFG]       = sizeof(struct msg_digital_config);
  msg_packed_len[MSG_VOLTAGE_CFG]       = sizeof(struct msg_voltage_config);
  msg_packed_len[MSG_TIME_DATA]         = sizeof(struct msg_time_data);
  msg_packed_len[MSG_ACCEL_DATA]        = sizeof(struct msg_accel_data);
  msg_packed_len[MSG_GYRO_DATA]         = sizeof(struct msg_gyro_data);
  msg_packed_len[MSG_MAG_DATA]          = sizeof(struct msg_mag_data);
  msg_packed_len[MSG_INCEPTOR_DATA]     = sizeof(struct msg_inceptor_data);
  msg_packed_len[MSG_TEMPERATURE_DATA]  = sizeof(struct msg_temperature_data);
  msg_packed_len[MSG_GNSS_DATA]         = sizeof(struct msg_gnss_data);
  msg_packed_len[MSG_STATIC_PRESS_DATA] = sizeof(struct msg_static_press_data);
  msg_packed_len[MSG_DIFF_PRESS_DATA]   = sizeof(struct msg_diff_press_data);
  msg_packed_len[MSG_ANALOG_DATA]       = sizeof(struct msg_analog_data);
  msg_packed_len[MSG_DIGITAL_DATA]      = sizeof(struct msg_digital_data);
  msg_packed_len[MSG_VOLTAGE_DATA]      = sizeof(struct msg_voltage_data);
  /* array of unpacked message lengths */
  msg_unpacked_len[MSG_FMU_CFG]           = sizeof(struct fmu_config);
  msg_unpacked_len[MSG_INT_MPU9250_CFG]   = sizeof(struct int_mpu9250_config);
  msg_unpacked_len[MSG_INT_BME280_CFG]    = sizeof(struct int_bme280_config);
  msg_unpacked_len[MSG_MPU9250_CFG]       = sizeof(struct mpu9250_config);
  msg_unpacked_len[MSG_BME280_CFG]        = sizeof(struct bme280_config);
  msg_unpacked_len[MSG_UBLOX_CFG]         = sizeof(struct ublox_config);
  msg_unpacked_len[MSG_SBUS_CFG]          = sizeof(struct sbus_config);
  msg_unpacked_len[MSG_SWIFT_CFG]         = sizeof(struct swift_config);
  msg_unpacked_len[MSG_AMS5915_CFG]       = sizeof(struct ams5915_config);
  msg_unpacked_len[MSG_ANALOG_CFG]        = sizeof(struct analog_config);
  msg_unpacked_len[MSG_DIGITAL_CFG]       = sizeof(struct digital_config);
  msg_unpacked_len[MSG_VOLTAGE_CFG]       = sizeof(struct voltage_config);
  msg_unpacked_len[MSG_TIME_DATA]         = sizeof(struct time_data);
  msg_unpacked_len[MSG_ACCEL_DATA]        = sizeof(struct accel_data);
  msg_unpacked_len[MSG_GYRO_DATA]         = sizeof(struct gyro_data);
  msg_unpacked_len[MSG_MAG_DATA]          = sizeof(struct mag_data);
  msg_unpacked_len[MSG_INCEPTOR_DATA]     = sizeof(struct inceptor_data);
  msg_unpacked_len[MSG_TEMPERATURE_DATA]  = sizeof(struct temperature_data);
  msg_unpacked_len[MSG_GNSS_DATA]         = sizeof(struct gnss_data);
  msg_unpacked_len[MSG_STATIC_PRESS_DATA] = sizeof(struct static_press_data);
  msg_unpacked_len[MSG_DIFF_PRESS_DATA]   = sizeof(struct diff_press_data);
  msg_unpacked_len[MSG_ANALOG_DATA]       = sizeof(struct analog_data);
  msg_unpacked_len[MSG_DIGITAL_DATA]      = sizeof(struct digital_data);
  msg_unpacked_len[MSG_VOLTAGE_DATA]      = sizeof(struct voltage_data);  
  /* array of function pointers to pack messages */
  pack_msg[MSG_FMU_CFG]           = pack_fmu_config;
  pack_msg[MSG_INT_MPU9250_CFG]   = pack_int_mpu9250_config;
  pack_msg[MSG_INT_BME280_CFG]    = pack_int_bme280_config;
  pack_msg[MSG_MPU9250_CFG]       = pack_mpu9250_config;
  pack_msg[MSG_BME280_CFG]        = pack_bme280_config;
  pack_msg[MSG_UBLOX_CFG]         = pack_ublox_config;
  pack_msg[MSG_SBUS_CFG]          = pack_sbus_config;
  pack_msg[MSG_SWIFT_CFG]         = pack_swift_config;
  pack_msg[MSG_AMS5915_CFG]       = pack_ams5915_config;
  pack_msg[MSG_ANALOG_CFG]        = pack_analog_config;
  pack_msg[MSG_DIGITAL_CFG]       = pack_digital_config;
  pack_msg[MSG_VOLTAGE_CFG]       = pack_voltage_config;
  pack_msg[MSG_TIME_DATA]         = pack_time_data;
  pack_msg[MSG_ACCEL_DATA]        = pack_accel_data;
  pack_msg[MSG_GYRO_DATA]         = pack_gyro_data;
  pack_msg[MSG_MAG_DATA]          = pack_mag_data;
  pack_msg[MSG_INCEPTOR_DATA]     = pack_inceptor_data;
  pack_msg[MSG_TEMPERATURE_DATA]  = pack_temperature_data;
  pack_msg[MSG_GNSS_DATA]         = pack_gnss_data;
  pack_msg[MSG_STATIC_PRESS_DATA] = pack_static_press_data;
  pack_msg[MSG_DIFF_PRESS_DATA]   = pack_diff_press_data;
  pack_msg[MSG_ANALOG_DATA]       = pack_analog_data;
  pack_msg[MSG_DIGITAL_DATA]      = pack_digital_data;
  pack_msg[MSG_VOLTAGE_DATA]      = pack_voltage_data;
  /* array of function pointers to unpack messages */
  unpack_msg[MSG_FMU_CFG]           = unpack_fmu_config;
  unpack_msg[MSG_INT_MPU9250_CFG]   = unpack_int_mpu9250_config;
  unpack_msg[MSG_INT_BME280_CFG]    = unpack_int_bme280_config;
  unpack_msg[MSG_MPU9250_CFG]       = unpack_mpu9250_config;
  unpack_msg[MSG_BME280_CFG]        = unpack_bme280_config;
  unpack_msg[MSG_UBLOX_CFG]         = unpack_ublox_config;
  unpack_msg[MSG_SBUS_CFG]          = unpack_sbus_config;
  unpack_msg[MSG_SWIFT_CFG]         = unpack_swift_config;
  unpack_msg[MSG_AMS5915_CFG]       = unpack_ams5915_config;
  unpack_msg[MSG_ANALOG_CFG]        = unpack_analog_config;
  unpack_msg[MSG_DIGITAL_CFG]       = unpack_digital_config;
  unpack_msg[MSG_VOLTAGE_CFG]       = unpack_voltage_config;
  unpack_msg[MSG_TIME_DATA]         = unpack_time_data;
  unpack_msg[MSG_ACCEL_DATA]        = unpack_accel_data;
  unpack_msg[MSG_GYRO_DATA]         = unpack_gyro_data;
  unpack_msg[MSG_MAG_DATA]          = unpack_mag_data;
  unpack_msg[MSG_INCEPTOR_DATA]     = unpack_inceptor_data;
  unpack_msg[MSG_TEMPERATURE_DATA]  = unpack_temperature_data;
  unpack_msg[MSG_GNSS_DATA]         = unpack_gnss_data;
  unpack_msg[MSG_STATIC_PRESS_DATA] = unpack_static_press_data;
  unpack_msg[MSG_DIFF_PRESS_DATA]   = unpack_diff_press_data;
  unpack_msg[MSG_ANALOG_DATA]       = unpack_analog_data;
  unpack_msg[MSG_DIGITAL_DATA]      = unpack_digital_data;
  unpack_msg[MSG_VOLTAGE_DATA]      = unpack_voltage_data;
  /* max packed message length (for sizing buffers) */
  msg_max_packed_len = 0;
  for (i = 0; i < ARRAY_SIZE(msg_packed_len); ++i) {
    if (msg_packed_len[i] > msg_max_packed_len) {
      msg_max_packed_len = msg_packed_len[i];
    }
  }
  /* max unpacked message length (for sizing buffers) */
  msg_max_unpacked_len = 0;
  for (i = 0; i < ARRAY_SIZE(msg_unpacked_len); ++i) {
    if (msg_unpacked_len[i] > msg_max_unpacked_len) {
      msg_max_unpacked_len = msg_unpacked_len[i];
    }
  }
  /* allocate the Msg_t structure */
  msg = calloc(1,sizeof(Msg_t));
  if (!msg) {return NULL;}
  /* allocate tx and rx buffers */
  msg->tx_buffer = malloc(msg_max_packed_len + MSG_HEADER_LEN + MSG_CHECKSUM_LEN);
  if (!msg->tx_buffer) {return NULL;}
  msg->tx_payload = malloc(msg_max_unpacked_len);
  if (!msg->tx_payload) {return NULL;}
  msg->rx_buffer = malloc(msg_max_packed_len + MSG_HEADER_LEN + MSG_CHECKSUM_LEN);
  if (!msg->rx_buffer) {return NULL;}
  msg->temp_rx_buffer = malloc(msg_max_packed_len + MSG_HEADER_LEN + MSG_CHECKSUM_LEN);
  if (!msg->temp_rx_buffer) {return NULL;}
  msg->rx_payload = malloc(msg_max_unpacked_len);
  if (!msg->rx_payload) {return NULL;}
  return msg;
}

MsgError_t msg_free(Msg_t *self)
{
  if (!self) {return MSG_ERROR_NULL_PTR;}
  free(self->tx_buffer);
  free(self->tx_payload);
  free(self->rx_buffer);
  free(self->temp_rx_buffer);
  free(self->rx_payload);
  return MSG_ERROR_SUCCESS;
}

MsgError_t msg_build_tx(Msg_t *self,unsigned int hash,MsgId_t msg,void *payload)
{
  unsigned char packed_buffer[msg_max_packed_len];
  unsigned short checksum;
  if ((!self) || (!payload)) {
    self->tx_status = MSG_ERROR_NULL_PTR;
    return MSG_ERROR_NULL_PTR;
  }
  if ((msg < 0) || (msg >= NUM_MSG_ID)) {
    self->tx_status = MSG_ERROR_INCORRECT_ID;
    return MSG_ERROR_INCORRECT_ID;    
  }
  /* header */
  self->tx_buffer[0] = 0x42;
  self->tx_buffer[1] = 0x46;
  /* hash */
  self->tx_buffer[2] = hash & 0xFF;
  self->tx_buffer[3] = (hash >> 8) & 0xFF;
  self->tx_buffer[4] = (hash >> 16) & 0xFF;
  self->tx_buffer[5] = (hash >> 24) & 0xFF;
  /* msg id */
  self->tx_buffer[6] = ((unsigned short) msg) & 0xFF;
  self->tx_buffer[7] = (((unsigned short) msg) >> 8) & 0xFF;  
  /* pack the payload */
  (*pack_msg[msg])(payload,(void *)packed_buffer);
  memcpy(self->tx_buffer + MSG_HEADER_LEN,packed_buffer,msg_packed_len[msg]);
  /* checksum */
  checksum = fletcher16(self->tx_buffer,MSG_HEADER_LEN + msg_packed_len[msg]);
  self->tx_buffer[MSG_HEADER_LEN + msg_packed_len[msg]] = checksum & 0xFF;
  self->tx_buffer[MSG_HEADER_LEN + msg_packed_len[msg] + 1] = (checksum >> 8) & 0xFF;
  /* meta data */
  self->tx_hash = hash;
  self->tx_id = msg;
  self->tx_len = msg_packed_len[msg] + MSG_HEADER_LEN + MSG_CHECKSUM_LEN;
  self->tx_status = MSG_ERROR_SUCCESS;
  memcpy(self->tx_payload,payload,msg_unpacked_len[msg]);
  return MSG_ERROR_SUCCESS;
}

MsgError_t msg_build_status(Msg_t *self)
{
  if (!self) {return MSG_ERROR_NULL_PTR;}
  return self->tx_status;
}

unsigned char *msg_tx_buffer(Msg_t *self)
{
  if (!self) {return NULL;}
  return self->tx_buffer;  
}

unsigned int msg_tx_len(Msg_t *self)
{
  if (!self) {return 0;}
  return self->tx_len;  
}

unsigned int msg_tx_hash(Msg_t *self)
{
  if (!self) {return 0;}
  return self->tx_hash;  
}

MsgId_t msg_tx_id(Msg_t *self)
{
  if (!self) {return 0;}
  return self->tx_id;  
}

void *msg_tx_payload(Msg_t *self)
{
  if (!self) {return NULL;}
  return self->tx_payload;    
}

MsgError_t msg_parse_rx(Msg_t *self,unsigned char byte)
{
  unsigned short computed_checksum, recv_checksum;
  if (!self) {
    self->rx_status = MSG_ERROR_NULL_PTR;
    return MSG_ERROR_NULL_PTR;    
  }
  /* sync on header */
  if (self->rx_state < ARRAY_SIZE(MSG_HEADER)) {
    if (byte == MSG_HEADER[self->rx_state]) {
      self->temp_rx_buffer[self->rx_state] = byte;
      ++self->rx_state;
    } else {
      self->rx_state = 0;
    }
  /* grab the hash and message id */
  } else if (self->rx_state < MSG_HEADER_LEN) {
    self->temp_rx_buffer[self->rx_state] = byte;
    ++self->rx_state;
  /* check the message id, grab the payload */
  } else if (self->rx_state == MSG_HEADER_LEN) {
    self->temp_rx_id = (MsgId_t)(self->temp_rx_buffer[MSG_HEADER_LEN - 1] << 8) | self->temp_rx_buffer[MSG_HEADER_LEN - 2];
    if ((self->temp_rx_id < 0) || (self->temp_rx_id >= NUM_MSG_ID)) {
      self->rx_status = MSG_ERROR_INCORRECT_ID;
      return MSG_ERROR_INCORRECT_ID;    
    }
    self->temp_rx_buffer[self->rx_state] = byte;
    ++self->rx_state;
  /* grab the payload and checksum */
  } else if (self->rx_state < MSG_HEADER_LEN + msg_packed_len[self->temp_rx_id] + MSG_CHECKSUM_LEN - 1) {
    self->temp_rx_buffer[self->rx_state] = byte;
    ++self->rx_state;
  /* compare checksum */
  } else if (self->rx_state < MSG_HEADER_LEN + msg_packed_len[self->temp_rx_id] + MSG_CHECKSUM_LEN) {
    self->temp_rx_buffer[self->rx_state] = byte;
    computed_checksum = fletcher16(self->temp_rx_buffer,MSG_HEADER_LEN + msg_packed_len[self->temp_rx_id]);
    recv_checksum = (unsigned short)(self->temp_rx_buffer[MSG_HEADER_LEN + msg_packed_len[self->temp_rx_id] + MSG_CHECKSUM_LEN - 1] << 8) | self->temp_rx_buffer[MSG_HEADER_LEN + msg_packed_len[self->temp_rx_id] + MSG_CHECKSUM_LEN - 2];
    if (computed_checksum == recv_checksum) {
      self->rx_id = self->temp_rx_id;
      self->rx_len = MSG_HEADER_LEN + msg_packed_len[self->rx_id] + MSG_CHECKSUM_LEN;
      memcpy(self->rx_buffer,self->temp_rx_buffer,self->rx_len);
      self->rx_hash = (unsigned int)(self->rx_buffer[MSG_HEADER_LEN - 3] << 24) | (self->rx_buffer[MSG_HEADER_LEN - 4] << 16) | (self->rx_buffer[MSG_HEADER_LEN - 5] << 8) | self->rx_buffer[MSG_HEADER_LEN - 6];
      (*unpack_msg[self->rx_id])(self->rx_buffer + MSG_HEADER_LEN,self->rx_payload);   
      self->rx_state = 0;   
      self->rx_status = MSG_ERROR_SUCCESS;
      return MSG_ERROR_SUCCESS;          
    } else {
      self->rx_state = 0;
      self->rx_status = MSG_ERROR_INCORRECT_CHECKSUM;
      return MSG_ERROR_INCORRECT_CHECKSUM;    
    }
  } else {
    self->rx_state = 0;
  }
  self->rx_status = MSG_ERROR_INCOMPLETE_MSG;
  return MSG_ERROR_INCOMPLETE_MSG;     
}

MsgError_t msg_parse_status(Msg_t *self)
{
  if (!self) {return MSG_ERROR_NULL_PTR;}
  return self->rx_status;
}

unsigned char *msg_rx_buffer(Msg_t *self)
{
  if (!self) {return NULL;}
  return self->rx_buffer;  
}

unsigned int msg_rx_len(Msg_t *self)
{
  if (!self) {return 0;}
  return self->rx_len;  
}

unsigned int msg_rx_hash(Msg_t *self)
{
  if (!self) {return 0;}
  return self->rx_hash;  
}

MsgId_t msg_rx_id(Msg_t *self)
{
  if (!self) {return 0;}
  return self->rx_id;  
}

void *msg_rx_payload(Msg_t *self)
{
  if (!self) {return NULL;}
  return self->rx_payload;    
}

void pack_fmu_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_fmu_config *) packed)->srd           = ((struct fmu_config *) unpacked)->srd;
    ((struct msg_fmu_config *) packed)->rot_yaw_rad   = ((struct fmu_config *) unpacked)->rot_yaw_rad * 511.5f / M_PI + 511.5f;
    ((struct msg_fmu_config *) packed)->rot_roll_rad  = ((struct fmu_config *) unpacked)->rot_roll_rad * 511.5f / M_PI + 511.5f;
    ((struct msg_fmu_config *) packed)->rot_pitch_rad = ((struct fmu_config *) unpacked)->rot_pitch_rad * 511.5f / M_PI + 255.5f;
  }
}

void unpack_fmu_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct fmu_config *)unpacked)->srd           = ((struct msg_fmu_config *)packed)->srd;
    ((struct fmu_config *)unpacked)->rot_yaw_rad   = (float) ((struct msg_fmu_config *)packed)->rot_yaw_rad * M_PI / 511.5f - M_PI;
    ((struct fmu_config *)unpacked)->rot_roll_rad  = (float) ((struct msg_fmu_config *)packed)->rot_roll_rad * M_PI / 511.5f - M_PI;
    ((struct fmu_config *)unpacked)->rot_pitch_rad = (float) ((struct msg_fmu_config *)packed)->rot_pitch_rad * M_PI / 511.5f - M_PI / 2.0f;
  }
}

void pack_int_mpu9250_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_int_mpu9250_config *)packed)->accel_range = ((struct int_mpu9250_config *)unpacked)->accel_range;
    ((struct msg_int_mpu9250_config *)packed)->gyro_range  = ((struct int_mpu9250_config *)unpacked)->gyro_range;
    ((struct msg_int_mpu9250_config *)packed)->bandwidth   = ((struct int_mpu9250_config *)unpacked)->bandwidth;
    ((struct msg_int_mpu9250_config *)packed)->ax_bias_mss = ((struct int_mpu9250_config *)unpacked)->ax_bias_mss * 1023.5f + 1023.5f;
    ((struct msg_int_mpu9250_config *)packed)->ay_bias_mss = ((struct int_mpu9250_config *)unpacked)->ax_bias_mss * 1023.5f + 1023.5f;
    ((struct msg_int_mpu9250_config *)packed)->az_bias_mss = ((struct int_mpu9250_config *)unpacked)->ax_bias_mss * 1023.5f + 1023.5f;
    ((struct msg_int_mpu9250_config *)packed)->hx_bias_ut  = ((struct int_mpu9250_config *)unpacked)->hx_bias_ut * 511.5f / 300.0f + 511.5f;
    ((struct msg_int_mpu9250_config *)packed)->hy_bias_ut  = ((struct int_mpu9250_config *)unpacked)->hy_bias_ut * 511.5f / 300.0f + 511.5f;
    ((struct msg_int_mpu9250_config *)packed)->hz_bias_ut  = ((struct int_mpu9250_config *)unpacked)->hz_bias_ut * 511.5f / 300.0f + 511.5f;
    ((struct msg_int_mpu9250_config *)packed)->hx_scale    = ((struct int_mpu9250_config *)unpacked)->hx_scale * 511.5f;
    ((struct msg_int_mpu9250_config *)packed)->hy_scale    = ((struct int_mpu9250_config *)unpacked)->hy_scale * 511.5f;
    ((struct msg_int_mpu9250_config *)packed)->hz_scale    = ((struct int_mpu9250_config *)unpacked)->hz_scale * 511.5f;
  }
}

void unpack_int_mpu9250_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct int_mpu9250_config *)unpacked)->accel_range = ((struct msg_int_mpu9250_config *)packed)->accel_range;
    ((struct int_mpu9250_config *)unpacked)->gyro_range  = ((struct msg_int_mpu9250_config *)packed)->gyro_range;
    ((struct int_mpu9250_config *)unpacked)->bandwidth   = ((struct msg_int_mpu9250_config *)packed)->bandwidth;
    ((struct int_mpu9250_config *)unpacked)->ax_bias_mss = (float) ((struct msg_int_mpu9250_config *)packed)->ax_bias_mss / 1023.5f - 1.0f;
    ((struct int_mpu9250_config *)unpacked)->ay_bias_mss = (float) ((struct msg_int_mpu9250_config *)packed)->ax_bias_mss / 1023.5f - 1.0f;
    ((struct int_mpu9250_config *)unpacked)->az_bias_mss = (float) ((struct msg_int_mpu9250_config *)packed)->ax_bias_mss / 1023.5f - 1.0f;
    ((struct int_mpu9250_config *)unpacked)->hx_bias_ut  = (float) ((struct msg_int_mpu9250_config *)packed)->hx_bias_ut * 300.0f / 511.5f - 300.0f;
    ((struct int_mpu9250_config *)unpacked)->hy_bias_ut  = (float) ((struct msg_int_mpu9250_config *)packed)->hy_bias_ut * 300.0f / 511.5f - 300.0f;
    ((struct int_mpu9250_config *)unpacked)->hz_bias_ut  = (float) ((struct msg_int_mpu9250_config *)packed)->hz_bias_ut * 300.0f / 511.5f - 300.0f;
    ((struct int_mpu9250_config *)unpacked)->hx_scale    = (float) ((struct msg_int_mpu9250_config *)packed)->hx_scale / 511.5f;
    ((struct int_mpu9250_config *)unpacked)->hy_scale    = (float) ((struct msg_int_mpu9250_config *)packed)->hy_scale / 511.5f;
    ((struct int_mpu9250_config *)unpacked)->hz_scale    = (float) ((struct msg_int_mpu9250_config *)packed)->hz_scale / 511.5f;
  }
}

void pack_int_bme280_config(void *unpacked, void *packed)
{}
void unpack_int_bme280_config(void *packed, void *unpacked)
{}

void pack_mpu9250_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_mpu9250_config *)packed)->bfs_addr    = ((struct mpu9250_config *)unpacked)->bfs_addr;
    ((struct msg_mpu9250_config *)packed)->use_spi     = ((struct mpu9250_config *)unpacked)->use_spi;
    ((struct msg_mpu9250_config *)packed)->bus         = (((struct mpu9250_config *)unpacked)->bus == 1) ? 0 : 1; 
    ((struct msg_mpu9250_config *)packed)->addr        = (((struct mpu9250_config *)unpacked)->addr == 0x68) ? 0 : 1;
    ((struct msg_mpu9250_config *)packed)->accel_range = ((struct mpu9250_config *)unpacked)->accel_range;
    ((struct msg_mpu9250_config *)packed)->gyro_range  = ((struct mpu9250_config *)unpacked)->gyro_range;
    ((struct msg_mpu9250_config *)packed)->srd         = ((struct mpu9250_config *)unpacked)->srd;
    ((struct msg_mpu9250_config *)packed)->bandwidth   = ((struct mpu9250_config *)unpacked)->bandwidth;
    ((struct msg_mpu9250_config *)packed)->ax_bias_mss = ((struct mpu9250_config *)unpacked)->ax_bias_mss * 1023.5f + 1023.5f;
    ((struct msg_mpu9250_config *)packed)->ay_bias_mss = ((struct mpu9250_config *)unpacked)->ax_bias_mss * 1023.5f + 1023.5f;
    ((struct msg_mpu9250_config *)packed)->az_bias_mss = ((struct mpu9250_config *)unpacked)->ax_bias_mss * 1023.5f + 1023.5f;
    ((struct msg_mpu9250_config *)packed)->hx_bias_ut  = ((struct mpu9250_config *)unpacked)->hx_bias_ut * 511.5f / 300.0f + 511.5f;
    ((struct msg_mpu9250_config *)packed)->hy_bias_ut  = ((struct mpu9250_config *)unpacked)->hy_bias_ut * 511.5f / 300.0f + 511.5f;
    ((struct msg_mpu9250_config *)packed)->hz_bias_ut  = ((struct mpu9250_config *)unpacked)->hz_bias_ut * 511.5f / 300.0f + 511.5f;
    ((struct msg_mpu9250_config *)packed)->hx_scale    = ((struct mpu9250_config *)unpacked)->hx_scale * 511.5f;
    ((struct msg_mpu9250_config *)packed)->hy_scale    = ((struct mpu9250_config *)unpacked)->hy_scale * 511.5f;
    ((struct msg_mpu9250_config *)packed)->hz_scale    = ((struct mpu9250_config *)unpacked)->hz_scale * 511.5f;
  }
}

void unpack_mpu9250_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct mpu9250_config *)unpacked)->bfs_addr    = ((struct msg_mpu9250_config *)packed)->bfs_addr;
    ((struct mpu9250_config *)unpacked)->use_spi     = ((struct msg_mpu9250_config *)packed)->use_spi;
    ((struct mpu9250_config *)unpacked)->bus         = (((struct msg_mpu9250_config *)packed)->bus == 0) ? 1 : 2; 
    ((struct mpu9250_config *)unpacked)->addr        = (((struct msg_mpu9250_config *)packed)->addr == 0) ? 0x68 : 0x69;
    ((struct mpu9250_config *)unpacked)->accel_range = ((struct msg_mpu9250_config *)packed)->accel_range;
    ((struct mpu9250_config *)unpacked)->gyro_range  = ((struct msg_mpu9250_config *)packed)->gyro_range;
    ((struct mpu9250_config *)unpacked)->srd         = ((struct msg_mpu9250_config *)packed)->srd;
    ((struct mpu9250_config *)unpacked)->bandwidth   = ((struct msg_mpu9250_config *)packed)->bandwidth;
    ((struct mpu9250_config *)unpacked)->ax_bias_mss = (float) ((struct msg_mpu9250_config *)packed)->ax_bias_mss / 1023.5f - 1.0f;
    ((struct mpu9250_config *)unpacked)->ay_bias_mss = (float) ((struct msg_mpu9250_config *)packed)->ax_bias_mss / 1023.5f - 1.0f;
    ((struct mpu9250_config *)unpacked)->az_bias_mss = (float) ((struct msg_mpu9250_config *)packed)->ax_bias_mss / 1023.5f - 1.0f;
    ((struct mpu9250_config *)unpacked)->hx_bias_ut  = (float) ((struct msg_mpu9250_config *)packed)->hx_bias_ut * 300.0f / 511.5f - 300.0f;
    ((struct mpu9250_config *)unpacked)->hy_bias_ut  = (float) ((struct msg_mpu9250_config *)packed)->hy_bias_ut * 300.0f / 511.5f - 300.0f;
    ((struct mpu9250_config *)unpacked)->hz_bias_ut  = (float) ((struct msg_mpu9250_config *)packed)->hz_bias_ut * 300.0f / 511.5f - 300.0f;
    ((struct mpu9250_config *)unpacked)->hx_scale    = (float) ((struct msg_mpu9250_config *)packed)->hx_scale / 511.5f;
    ((struct mpu9250_config *)unpacked)->hy_scale    = (float) ((struct msg_mpu9250_config *)packed)->hy_scale / 511.5f;
    ((struct mpu9250_config *)unpacked)->hz_scale    = (float) ((struct msg_mpu9250_config *)packed)->hz_scale / 511.5f;
  }
}

void pack_bme280_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_bme280_config *)packed)->bfs_addr    = ((struct bme280_config *)unpacked)->bfs_addr;
    ((struct msg_bme280_config *)packed)->use_spi     = ((struct bme280_config *)unpacked)->use_spi;
    ((struct msg_bme280_config *)packed)->bus         = (((struct bme280_config *)unpacked)->bus == 1) ? 0 : 1; 
    ((struct msg_bme280_config *)packed)->addr        = (((struct bme280_config *)unpacked)->addr == 0x76) ? 0 : 1;
  }
}

void unpack_bme280_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct bme280_config *)unpacked)->bfs_addr    = ((struct msg_bme280_config *)packed)->bfs_addr;
    ((struct bme280_config *)unpacked)->use_spi     = ((struct msg_bme280_config *)packed)->use_spi;
    ((struct bme280_config *)unpacked)->bus         = (((struct msg_bme280_config *)packed)->bus == 0) ? 1 : 2; 
    ((struct bme280_config *)unpacked)->addr        = (((struct msg_bme280_config *)packed)->addr == 0) ? 0x76 : 0x77;
  }
}

void pack_ublox_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
      ((struct msg_ublox_config *)packed)->bfs_addr  = ((struct msg_ublox_config *)unpacked)->bfs_addr;
      ((struct msg_ublox_config *)packed)->uart      = ((struct msg_ublox_config *)unpacked)->uart;
    if (((struct msg_ublox_config *)unpacked)->baud > 460800) {
      ((struct msg_ublox_config *)packed)->baud      = 7;
    } else if (((struct msg_ublox_config *)unpacked)->baud > 230400) {
      ((struct msg_ublox_config *)packed)->baud      = 6;
    } else if (((struct msg_ublox_config *)unpacked)->baud > 115200) {
      ((struct msg_ublox_config *)packed)->baud      = 5;
    } else if (((struct msg_ublox_config *)unpacked)->baud > 57600) {
      ((struct msg_ublox_config *)packed)->baud      = 4;
    } else if (((struct msg_ublox_config *)unpacked)->baud > 38400) {
      ((struct msg_ublox_config *)packed)->baud      = 3;
    } else if (((struct msg_ublox_config *)unpacked)->baud > 19200) {
      ((struct msg_ublox_config *)packed)->baud      = 2;
    } else if (((struct msg_ublox_config *)unpacked)->baud > 9600) {
      ((struct msg_ublox_config *)packed)->baud      = 1;
    } else {
      ((struct msg_ublox_config *)packed)->baud      = 0;
    }
  }
}

void unpack_ublox_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
      ((struct ublox_config *)unpacked)->bfs_addr  = ((struct msg_ublox_config *)packed)->bfs_addr;
      ((struct ublox_config *)unpacked)->uart      = ((struct msg_ublox_config *)packed)->uart;
    switch (((struct msg_ublox_config *)packed)->baud) {
      case 0: {
        ((struct ublox_config *)unpacked)->baud    = 9600;
        break;
      }
      case 1: {
        ((struct ublox_config *)unpacked)->baud    = 19200;
        break;
      }
      case 2: {
        ((struct ublox_config *)unpacked)->baud    = 38400;
        break;
      }
      case 3: {
        ((struct ublox_config *)unpacked)->baud    = 57600;
        break;
      }
      case 4: {
        ((struct ublox_config *)unpacked)->baud    = 115200;
        break;
      }
      case 5: {
        ((struct ublox_config *)unpacked)->baud    = 230400;
        break;
      }
      case 6: {
        ((struct ublox_config *)unpacked)->baud    = 460800;
        break;
      }
      case 7: {
        ((struct ublox_config *)unpacked)->baud    = 921600;
        break;
      }
      default: {
        ((struct ublox_config *)unpacked)->baud    = 115200;
        break;        
      }
    }
  }
}

void pack_sbus_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_sbus_config *)packed)->bfs_addr = ((struct sbus_config *)unpacked)->bfs_addr;
  }
}

void unpack_sbus_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct sbus_config *)unpacked)->bfs_addr = ((struct msg_sbus_config *)packed)->bfs_addr;
  }
}

void pack_swift_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_swift_config *)packed)->bfs_addr      = ((struct swift_config *)unpacked)->bfs_addr;
    ((struct msg_swift_config *)packed)->bus           = (((struct swift_config *)unpacked)->bus == 1) ? 0 : 1; 
    ((struct msg_swift_config *)packed)->static_addr   = ((struct swift_config *)unpacked)->static_addr;
    ((struct msg_swift_config *)packed)->diff_addr     = ((struct swift_config *)unpacked)->diff_addr;
    switch (((struct swift_config *)unpacked)->diff_type) {
      case 0: {
        ((struct msg_swift_config *)packed)->diff_type = 0;
      }
      case 1: {
        ((struct msg_swift_config *)packed)->diff_type = 1;
      }
      case 4: {
        ((struct msg_swift_config *)packed)->diff_type = 2;
      }
    }
  }
}

void unpack_swift_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct swift_config *)unpacked)->bfs_addr      = ((struct msg_swift_config *)packed)->bfs_addr;
    ((struct swift_config *)unpacked)->bus           = (((struct msg_swift_config *)packed)->bus == 0) ? 1 : 2; 
    ((struct swift_config *)unpacked)->static_addr   = ((struct msg_swift_config *)packed)->static_addr;
    ((struct swift_config *)unpacked)->diff_addr     = ((struct msg_swift_config *)packed)->diff_addr;
    switch (((struct msg_swift_config *)packed)->diff_type) {
      case 0: {
        ((struct swift_config *)unpacked)->diff_type = 0;
      }
      case 1: {
        ((struct swift_config *)unpacked)->diff_type = 1;
      }
      case 2: {
        ((struct swift_config *)unpacked)->diff_type = 4;
      }
    }
  }
}

void pack_ams5915_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_ams5915_config *)packed)->bfs_addr      = ((struct ams5915_config *)unpacked)->bfs_addr;
    ((struct msg_ams5915_config *)packed)->bus           = (((struct ams5915_config *)unpacked)->bus == 1) ? 0 : 1; 
    ((struct msg_ams5915_config *)packed)->addr          = ((struct ams5915_config *)unpacked)->addr;
    ((struct msg_ams5915_config *)packed)->type          = ((struct ams5915_config *)unpacked)->type;
  }
}

void unpack_ams5915_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct ams5915_config *)unpacked)->bfs_addr      = ((struct msg_ams5915_config *)packed)->bfs_addr;
    ((struct ams5915_config *)unpacked)->bus           = (((struct msg_ams5915_config *)packed)->bus == 0) ? 1 : 2; 
    ((struct ams5915_config *)unpacked)->addr          = ((struct msg_ams5915_config *)packed)->addr;
    ((struct ams5915_config *)unpacked)->type          = ((struct msg_ams5915_config *)packed)->type;
  }
}

void pack_analog_config(void *unpacked, void *packed)
{
  unsigned int i, temp[8];
  if ((unpacked) && (packed)) {
    ((struct msg_analog_config *)packed)->bfs_addr      = ((struct analog_config *)unpacked)->bfs_addr;
    ((struct msg_analog_config *)packed)->ch            = ((struct analog_config *)unpacked)->ch; 
    ((struct msg_analog_config *)packed)->coeff_len     = ((struct analog_config *)unpacked)->coeff_len;
    for (i = 0; i < ARRAY_SIZE(temp); ++i) {
      memcpy(&temp[i],&((struct analog_config *)unpacked)->coeff[i],sizeof(temp[i]));
    }
    ((struct msg_analog_config *)packed)->coeff0        = temp[0];
    ((struct msg_analog_config *)packed)->coeff1        = temp[1];
    ((struct msg_analog_config *)packed)->coeff2        = temp[2];
    ((struct msg_analog_config *)packed)->coeff3        = temp[3];
    ((struct msg_analog_config *)packed)->coeff4        = temp[4];
    ((struct msg_analog_config *)packed)->coeff5        = temp[5];
    ((struct msg_analog_config *)packed)->coeff6        = temp[6];
    ((struct msg_analog_config *)packed)->coeff7        = temp[7];
  }
}

void unpack_analog_config(void *packed, void *unpacked)
{
  unsigned int i, temp[8];
  if ((unpacked) && (packed)) {
    ((struct analog_config *)unpacked)->bfs_addr    = ((struct msg_analog_config *)packed)->bfs_addr;
    ((struct analog_config *)unpacked)->ch          = ((struct msg_analog_config *)packed)->ch; 
    ((struct analog_config *)unpacked)->coeff_len   = ((struct msg_analog_config *)packed)->coeff_len;
    temp[0]                                         = ((struct msg_analog_config *)packed)->coeff0;
    temp[1]                                         = ((struct msg_analog_config *)packed)->coeff1;
    temp[2]                                         = ((struct msg_analog_config *)packed)->coeff2;
    temp[3]                                         = ((struct msg_analog_config *)packed)->coeff3;
    temp[4]                                         = ((struct msg_analog_config *)packed)->coeff4;
    temp[5]                                         = ((struct msg_analog_config *)packed)->coeff5;
    temp[6]                                         = ((struct msg_analog_config *)packed)->coeff6;
    temp[7]                                         = ((struct msg_analog_config *)packed)->coeff7;
    for (i = 0; i < ARRAY_SIZE(temp); ++i) {
      memcpy(&((struct analog_config *)unpacked)->coeff[i],&temp[i],sizeof(((struct analog_config *)unpacked)->coeff[i]));
    }
  }
}

void pack_digital_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_digital_config *)packed)->bfs_addr      = ((struct digital_config *)unpacked)->bfs_addr;
    ((struct msg_digital_config *)packed)->ch            = ((struct digital_config *)unpacked)->ch; 
    ((struct msg_digital_config *)packed)->active_high   = ((struct digital_config *)unpacked)->active_high;
  }
}

void unpack_digital_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct digital_config *)unpacked)->bfs_addr      = ((struct msg_digital_config *)packed)->bfs_addr;
    ((struct digital_config *)unpacked)->ch            = ((struct msg_digital_config *)packed)->ch; 
    ((struct digital_config *)unpacked)->active_high   = ((struct msg_digital_config *)packed)->active_high;
  }
}

void pack_voltage_config(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_voltage_config *)packed)->bfs_addr      = ((struct voltage_config *)unpacked)->bfs_addr;
    ((struct msg_voltage_config *)packed)->type          = ((struct voltage_config *)unpacked)->type;
  }
}

void unpack_voltage_config(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct voltage_config *)unpacked)->bfs_addr      = ((struct msg_voltage_config *)packed)->bfs_addr;
    ((struct voltage_config *)unpacked)->type          = ((struct msg_voltage_config *)packed)->type;
  }
}

void pack_time_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_time_data *)packed)->frame_counter = ((struct time_data *)unpacked)->frame_counter;
    ((struct msg_time_data *)packed)->frames        = ((struct time_data *)unpacked)->frames;
  }
}

void unpack_time_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct time_data *)unpacked)->frame_counter = ((struct msg_time_data *)packed)->frame_counter;
    ((struct time_data *)unpacked)->frames        = ((struct msg_time_data *)packed)->frames;
  }
}

void pack_accel_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_accel_data *)packed)->frame_counter   = ((struct accel_data *)unpacked)->frame_counter;
    ((struct msg_accel_data *)packed)->time_offset_us  = ((struct accel_data *)unpacked)->time_offset_us;
    ((struct msg_accel_data *)packed)->accel_x_mss     = ((struct accel_data *)unpacked)->accel_x_mss * 262143.5f / (16.0f * G) + 262143.5f;
    ((struct msg_accel_data *)packed)->accel_y_mss     = ((struct accel_data *)unpacked)->accel_y_mss * 262143.5f / (16.0f * G) + 262143.5f;
    ((struct msg_accel_data *)packed)->accel_z_mss     = ((struct accel_data *)unpacked)->accel_z_mss * 262143.5f / (16.0f * G) + 262143.5f;
  }
}

void unpack_accel_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct accel_data *)unpacked)->frame_counter   = ((struct msg_accel_data *)packed)->frame_counter;
    ((struct accel_data *)unpacked)->time_offset_us  = ((struct msg_accel_data *)packed)->time_offset_us;
    ((struct accel_data *)unpacked)->accel_x_mss     = (float) ((struct msg_accel_data *)packed)->accel_x_mss * 16.0f * G / 262143.5f - 16.0f * G;
    ((struct accel_data *)unpacked)->accel_y_mss     = (float) ((struct msg_accel_data *)packed)->accel_y_mss * 16.0f * G / 262143.5f - 16.0f * G;
    ((struct accel_data *)unpacked)->accel_z_mss     = (float) ((struct msg_accel_data *)packed)->accel_z_mss * 16.0f * G / 262143.5f - 16.0f * G;
  }
}

void pack_gyro_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_gyro_data *)packed)->frame_counter   = ((struct gyro_data *)unpacked)->frame_counter;
    ((struct msg_gyro_data *)packed)->time_offset_us  = ((struct gyro_data *)unpacked)->time_offset_us;
    ((struct msg_gyro_data *)packed)->gyro_x_rads     = ((struct gyro_data *)unpacked)->gyro_x_rads * 2097151.5f / (2000.0f * D2R);
    ((struct msg_gyro_data *)packed)->gyro_y_rads     = ((struct gyro_data *)unpacked)->gyro_y_rads * 2097151.5f / (2000.0f * D2R);
    ((struct msg_gyro_data *)packed)->gyro_z_rads     = ((struct gyro_data *)unpacked)->gyro_z_rads * 2097151.5f / (2000.0f * D2R);
  }
}

void unpack_gyro_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct gyro_data *)unpacked)->frame_counter   = ((struct msg_gyro_data *)packed)->frame_counter;
    ((struct gyro_data *)unpacked)->time_offset_us  = ((struct msg_gyro_data *)packed)->time_offset_us;
    ((struct gyro_data *)unpacked)->gyro_x_rads     = (float) ((struct msg_gyro_data *)packed)->gyro_x_rads * 2000.0f * D2R / 2097151.5f - 2000.0f * D2R;
    ((struct gyro_data *)unpacked)->gyro_y_rads     = (float) ((struct msg_gyro_data *)packed)->gyro_y_rads * 2000.0f * D2R / 2097151.5f - 2000.0f * D2R;
    ((struct gyro_data *)unpacked)->gyro_z_rads     = (float) ((struct msg_gyro_data *)packed)->gyro_z_rads * 2000.0f * D2R / 2097151.5f - 2000.0f * D2R;
  }
}

void pack_mag_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_mag_data *)packed)->frame_counter   = ((struct mag_data *)unpacked)->frame_counter;
    ((struct msg_mag_data *)packed)->time_offset_us  = ((struct mag_data *)unpacked)->time_offset_us;
    ((struct msg_mag_data *)packed)->mag_x_ut        = ((struct mag_data *)unpacked)->mag_x_ut * 131071.5f / 1000.0f + 131071.5f; 
    ((struct msg_mag_data *)packed)->mag_y_ut        = ((struct mag_data *)unpacked)->mag_y_ut * 131071.5f / 1000.0f + 131071.5f; 
    ((struct msg_mag_data *)packed)->mag_z_ut        = ((struct mag_data *)unpacked)->mag_z_ut * 131071.5f / 1000.0f + 131071.5f; 
  }
}

void unpack_mag_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct mag_data *)unpacked)->frame_counter   = ((struct msg_mag_data *)packed)->frame_counter;
    ((struct mag_data *)unpacked)->time_offset_us  = ((struct msg_mag_data *)packed)->time_offset_us;
    ((struct mag_data *)unpacked)->mag_x_ut     = (float) ((struct msg_mag_data *)packed)->mag_x_ut * 1000.0f / 131071.5f - 1000.0f;
    ((struct mag_data *)unpacked)->mag_y_ut     = (float) ((struct msg_mag_data *)packed)->mag_y_ut * 1000.0f / 131071.5f - 1000.0f;
    ((struct mag_data *)unpacked)->mag_z_ut     = (float) ((struct msg_mag_data *)packed)->mag_z_ut * 1000.0f / 131071.5f - 1000.0f;
  }
}

void pack_inceptor_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_inceptor_data *)packed)->frame_counter       = ((struct inceptor_data *)unpacked)->frame_counter;
    ((struct msg_inceptor_data *)packed)->time_offset_us      = ((struct inceptor_data *)unpacked)->time_offset_us;
    ((struct msg_inceptor_data *)packed)->lost_frame          = ((struct inceptor_data *)unpacked)->lost_frame;
    ((struct msg_inceptor_data *)packed)->failsafe_activated  = ((struct inceptor_data *)unpacked)->failsafe_activated;
    ((struct msg_inceptor_data *)packed)->ch0                 = ((struct inceptor_data *)unpacked)->ch[0] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch1                 = ((struct inceptor_data *)unpacked)->ch[1] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch2                 = ((struct inceptor_data *)unpacked)->ch[2] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch3                 = ((struct inceptor_data *)unpacked)->ch[3] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch4                 = ((struct inceptor_data *)unpacked)->ch[4] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch5                 = ((struct inceptor_data *)unpacked)->ch[5] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch6                 = ((struct inceptor_data *)unpacked)->ch[6] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch7                 = ((struct inceptor_data *)unpacked)->ch[7] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch8                 = ((struct inceptor_data *)unpacked)->ch[8] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch9                 = ((struct inceptor_data *)unpacked)->ch[9] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch10                = ((struct inceptor_data *)unpacked)->ch[10] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch11                = ((struct inceptor_data *)unpacked)->ch[11] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch12                = ((struct inceptor_data *)unpacked)->ch[12] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch13                = ((struct inceptor_data *)unpacked)->ch[13] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch14                = ((struct inceptor_data *)unpacked)->ch[14] * 1023.5f + 1023.5f;
    ((struct msg_inceptor_data *)packed)->ch15                = ((struct inceptor_data *)unpacked)->ch[15] * 1023.5f + 1023.5f;
  }
}

void unpack_inceptor_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct inceptor_data *)unpacked)->frame_counter       = ((struct msg_inceptor_data *)packed)->frame_counter;
    ((struct inceptor_data *)unpacked)->time_offset_us      = ((struct msg_inceptor_data *)packed)->time_offset_us;
    ((struct inceptor_data *)unpacked)->lost_frame          = ((struct msg_inceptor_data *)packed)->lost_frame;
    ((struct inceptor_data *)unpacked)->failsafe_activated  = ((struct msg_inceptor_data *)packed)->failsafe_activated;
    ((struct inceptor_data *)unpacked)->ch[0]               = (float) ((struct msg_inceptor_data *)packed)->ch0 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[1]               = (float) ((struct msg_inceptor_data *)packed)->ch1 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[2]               = (float) ((struct msg_inceptor_data *)packed)->ch2 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[3]               = (float) ((struct msg_inceptor_data *)packed)->ch3 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[4]               = (float) ((struct msg_inceptor_data *)packed)->ch4 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[5]               = (float) ((struct msg_inceptor_data *)packed)->ch5 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[6]               = (float) ((struct msg_inceptor_data *)packed)->ch6 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[7]               = (float) ((struct msg_inceptor_data *)packed)->ch7 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[8]               = (float) ((struct msg_inceptor_data *)packed)->ch8 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[9]               = (float) ((struct msg_inceptor_data *)packed)->ch9 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[10]              = (float) ((struct msg_inceptor_data *)packed)->ch10 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[11]              = (float) ((struct msg_inceptor_data *)packed)->ch11 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[12]              = (float) ((struct msg_inceptor_data *)packed)->ch12 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[13]              = (float) ((struct msg_inceptor_data *)packed)->ch13 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[14]              = (float) ((struct msg_inceptor_data *)packed)->ch14 / 1023.5f - 1.0f;
    ((struct inceptor_data *)unpacked)->ch[15]              = (float) ((struct msg_inceptor_data *)packed)->ch15 / 1023.5f - 1.0f;
  }
}

void pack_temperature_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_temperature_data *)packed)->frame_counter   = ((struct temperature_data *)unpacked)->frame_counter;
    ((struct msg_temperature_data *)packed)->time_offset_us  = ((struct temperature_data *)unpacked)->time_offset_us;
    ((struct msg_temperature_data *)packed)->temp_c          = ((struct temperature_data *)unpacked)->temp_c * 16383.0f / 120.0f + 5461.0f;
  }
}

void unpack_temperature_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct temperature_data *)unpacked)->frame_counter   = ((struct msg_temperature_data *)packed)->frame_counter;
    ((struct temperature_data *)unpacked)->time_offset_us  = ((struct msg_temperature_data *)packed)->time_offset_us;
    ((struct temperature_data *)unpacked)->temp_c          = (float) ((struct msg_temperature_data *)packed)->temp_c * 120.0f / 16383.0f - 40.0f;
  }
}

void pack_gnss_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_gnss_data *)packed)->frame_counter   = ((struct gnss_data *)unpacked)->frame_counter;
    ((struct msg_gnss_data *)packed)->time_offset_us  = ((struct gnss_data *)unpacked)->time_offset_us;
    ((struct msg_gnss_data *)packed)->fix             = ((struct gnss_data *)unpacked)->fix;
    ((struct msg_gnss_data *)packed)->num_sv          = ((struct gnss_data *)unpacked)->num_sv;
    ((struct msg_gnss_data *)packed)->tow_ms          = ((struct gnss_data *)unpacked)->tow_ms;
    ((struct msg_gnss_data *)packed)->lat_rad         = ((struct gnss_data *)unpacked)->lat_rad * 2147483647.5 / D_PI + 2147483647.5;
    ((struct msg_gnss_data *)packed)->lon_rad         = ((struct gnss_data *)unpacked)->lon_rad * 2147483647.5 / D_PI + 2147483647.5;
    ((struct msg_gnss_data *)packed)->hmsl_m          = ((struct gnss_data *)unpacked)->hmsl_m * 1118.48105f + 11184810.5f;
    ((struct msg_gnss_data *)packed)->vel_north_ms    = ((struct gnss_data *)unpacked)->vel_north_ms * 1048.575f + 524287.5f;
    ((struct msg_gnss_data *)packed)->vel_east_ms     = ((struct gnss_data *)unpacked)->vel_east_ms * 1048.575f + 524287.5f;
    ((struct msg_gnss_data *)packed)->vel_down_ms     = ((struct gnss_data *)unpacked)->vel_down_ms * 1310.715f + 131071.5f;
    ((struct msg_gnss_data *)packed)->horiz_acc_m     = ((struct gnss_data *)unpacked)->horiz_acc_m * 1310.71f;
    ((struct msg_gnss_data *)packed)->vert_acc_m      = ((struct gnss_data *)unpacked)->vert_acc_m * 1310.71f;
    ((struct msg_gnss_data *)packed)->speed_acc_ms    = ((struct gnss_data *)unpacked)->speed_acc_ms * 1310.71f;
    ((struct msg_gnss_data *)packed)->pdop            = ((struct gnss_data *)unpacked)->pdop * 163.83f;
  }
}

void unpack_gnss_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct gnss_data *)unpacked)->frame_counter   = ((struct msg_gnss_data *)packed)->frame_counter;
    ((struct gnss_data *)unpacked)->time_offset_us  = ((struct msg_gnss_data *)packed)->time_offset_us;
    ((struct gnss_data *)unpacked)->fix             = ((struct msg_gnss_data *)packed)->fix;
    ((struct gnss_data *)unpacked)->num_sv          = ((struct msg_gnss_data *)packed)->num_sv;
    ((struct gnss_data *)unpacked)->tow_ms          = ((struct msg_gnss_data *)packed)->tow_ms;
    ((struct gnss_data *)unpacked)->lat_rad         = (double) ((struct msg_gnss_data *)packed)->lat_rad * D_PI / 2147483647.5 - D_PI;
    ((struct gnss_data *)unpacked)->lon_rad         = (double) ((struct msg_gnss_data *)packed)->lon_rad * D_PI / 2147483647.5 - D_PI;
    ((struct gnss_data *)unpacked)->hmsl_m          = (float) ((struct msg_gnss_data *)packed)->hmsl_m * 60000.0f / 67108863.0f - 10000.0f;
    ((struct gnss_data *)unpacked)->vel_north_ms    = (float) ((struct msg_gnss_data *)packed)->vel_north_ms * 500.0f / 524287.5f - 500.0f;
    ((struct gnss_data *)unpacked)->vel_east_ms     = (float) ((struct msg_gnss_data *)packed)->vel_east_ms * 500.0f / 524287.5f - 500.0f;
    ((struct gnss_data *)unpacked)->vel_down_ms     = (float) ((struct msg_gnss_data *)packed)->vel_down_ms * 100.0f / 131071.5f - 100.0f;
    ((struct gnss_data *)unpacked)->horiz_acc_m     = (float) ((struct msg_gnss_data *)packed)->horiz_acc_m / 1310.71f;
    ((struct gnss_data *)unpacked)->vert_acc_m      = (float) ((struct msg_gnss_data *)packed)->vert_acc_m / 1310.71f;
    ((struct gnss_data *)unpacked)->speed_acc_ms    = (float) ((struct msg_gnss_data *)packed)->speed_acc_ms / 1310.71f;
    ((struct gnss_data *)unpacked)->pdop            = (float) ((struct msg_gnss_data *)packed)->pdop / 163.83f;
  }
}

void pack_static_press_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_static_press_data *)packed)->frame_counter   = ((struct static_press_data *)unpacked)->frame_counter;
    ((struct msg_static_press_data *)packed)->time_offset_us  = ((struct static_press_data *)unpacked)->time_offset_us;
    ((struct msg_static_press_data *)packed)->press_pa        = ((struct static_press_data *)unpacked)->press_pa * 4194303.0f / 80000.0f - 1572863.625f;
  }
}

void unpack_static_press_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct static_press_data *)unpacked)->frame_counter   = ((struct msg_static_press_data *)packed)->frame_counter;
    ((struct static_press_data *)unpacked)->time_offset_us  = ((struct msg_static_press_data *)packed)->time_offset_us;
    ((struct static_press_data *)unpacked)->press_pa        = (float) ((struct msg_static_press_data *)packed)->press_pa * 80000.0f / 4194303.0f + 30000.0f; 
  }
}

void pack_diff_press_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_diff_press_data *)packed)->frame_counter   = ((struct diff_press_data *)unpacked)->frame_counter;
    ((struct msg_diff_press_data *)packed)->time_offset_us  = ((struct diff_press_data *)unpacked)->time_offset_us;
    ((struct msg_diff_press_data *)packed)->press_pa        = ((struct diff_press_data *)unpacked)->press_pa * 2097151.5f / 20000.0f + 2097151.5f;
  }
}

void unpack_diff_press_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct diff_press_data *)unpacked)->frame_counter   = ((struct msg_diff_press_data *)packed)->frame_counter;
    ((struct diff_press_data *)unpacked)->time_offset_us  = ((struct msg_diff_press_data *)packed)->time_offset_us;
    ((struct diff_press_data *)unpacked)->press_pa        = (float) ((struct msg_diff_press_data *)packed)->press_pa * 20000.0f / 2097151.5f - 20000.0f;
  }
}

void pack_analog_data(void *unpacked, void *packed)
{
  unsigned int temp;
  if ((unpacked) && (packed)) {
    ((struct msg_analog_data *)packed)->frame_counter   = ((struct analog_data *)unpacked)->frame_counter;
    ((struct msg_analog_data *)packed)->time_offset_us  = ((struct analog_data *)unpacked)->time_offset_us;
    ((struct msg_analog_data *)packed)->voltage         = ((struct analog_data *)unpacked)->voltage * 8191.0f / 3.3f;
    memcpy(&temp,&((struct analog_data *)unpacked)->cal_value,sizeof(temp));
    ((struct msg_analog_data *)packed)->cal_value       = temp;
  }
}

void unpack_analog_data(void *packed, void *unpacked)
{
  unsigned int temp;
  if ((unpacked) && (packed)) {
    ((struct analog_data *)unpacked)->frame_counter   = ((struct msg_analog_data *)packed)->frame_counter;
    ((struct analog_data *)unpacked)->time_offset_us  = ((struct msg_analog_data *)packed)->time_offset_us;
    ((struct analog_data *)unpacked)->voltage         = (float) ((struct msg_analog_data *)packed)->voltage * 3.3f / 8191.0f;
    temp                                              = ((struct msg_analog_data *)packed)->cal_value;
    memcpy(&((struct analog_data *)unpacked)->cal_value,&temp,sizeof(((struct analog_data *)unpacked)->cal_value));
  }
}

void pack_digital_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_digital_data *)packed)->frame_counter   = ((struct digital_data *)unpacked)->frame_counter;
    ((struct msg_digital_data *)packed)->time_offset_us  = ((struct digital_data *)unpacked)->time_offset_us;
    ((struct msg_digital_data *)packed)->digout          = ((struct digital_data *)unpacked)->digout;
  }
}

void unpack_digital_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct digital_data *)unpacked)->frame_counter   = ((struct msg_digital_data *)packed)->frame_counter;
    ((struct digital_data *)unpacked)->time_offset_us  = ((struct msg_digital_data *)packed)->time_offset_us;
    ((struct digital_data *)unpacked)->digout          = ((struct msg_digital_data *)packed)->digout;
  }
}

void pack_voltage_data(void *unpacked, void *packed)
{
  if ((unpacked) && (packed)) {
    ((struct msg_voltage_data *)packed)->frame_counter   = ((struct voltage_data *)unpacked)->frame_counter;
    ((struct msg_voltage_data *)packed)->time_offset_us  = ((struct voltage_data *)unpacked)->time_offset_us;
    ((struct msg_voltage_data *)packed)->voltage         = ((struct voltage_data *)unpacked)->voltage * 8191.0f / 36.0f;
  }
}

void unpack_voltage_data(void *packed, void *unpacked)
{
  if ((unpacked) && (packed)) {
    ((struct voltage_data *)unpacked)->frame_counter   = ((struct msg_voltage_data *)packed)->frame_counter;
    ((struct voltage_data *)unpacked)->time_offset_us  = ((struct msg_voltage_data *)packed)->time_offset_us;
    ((struct voltage_data *)unpacked)->voltage         = (float) ((struct msg_voltage_data *)packed)->voltage * 36.0f / 8191.0f;
  }
}
