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

void pack_fmu_config(struct fmu_config *unpacked, struct msg_fmu_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->srd           = unpacked->srd;
    packed->rot_yaw_rad   = unpacked->rot_yaw_rad * 511.5f / M_PI + 511.5f;
    packed->rot_roll_rad  = unpacked->rot_roll_rad * 511.5f / M_PI + 511.5f;
    packed->rot_pitch_rad = unpacked->rot_pitch_rad * 511.5f / M_PI + 255.5f;
  }
}

void unpack_fmu_config(struct msg_fmu_config *packed, struct fmu_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->srd           = packed->srd;
    unpacked->rot_yaw_rad   = (float) packed->rot_yaw_rad * M_PI / 511.5f - M_PI;
    unpacked->rot_roll_rad  = (float) packed->rot_roll_rad * M_PI / 511.5f - M_PI;
    unpacked->rot_pitch_rad = (float) packed->rot_pitch_rad * M_PI / 511.5f - M_PI / 2.0f;
  }
}

void pack_int_mpu9250_config(struct int_mpu9250_config *unpacked, struct msg_int_mpu9250_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->accel_range = unpacked->accel_range;
    packed->gyro_range  = unpacked->gyro_range;
    packed->bandwidth   = unpacked->bandwidth;
    packed->ax_bias_mss = unpacked->ax_bias_mss * 1023.5f + 1023.5f;
    packed->ay_bias_mss = unpacked->ax_bias_mss * 1023.5f + 1023.5f;
    packed->az_bias_mss = unpacked->ax_bias_mss * 1023.5f + 1023.5f;
    packed->hx_bias_ut  = unpacked->hx_bias_ut * 511.5f / 300.0f + 511.5f;
    packed->hy_bias_ut  = unpacked->hy_bias_ut * 511.5f / 300.0f + 511.5f;
    packed->hz_bias_ut  = unpacked->hz_bias_ut * 511.5f / 300.0f + 511.5f;
    packed->hx_scale    = unpacked->hx_scale * 511.5f;
    packed->hy_scale    = unpacked->hy_scale * 511.5f;
    packed->hz_scale    = unpacked->hz_scale * 511.5f;
  }
}

void unpack_int_mpu9250_config(struct msg_int_mpu9250_config *packed, struct int_mpu9250_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->accel_range = packed->accel_range;
    unpacked->gyro_range  = packed->gyro_range;
    unpacked->bandwidth   = packed->bandwidth;
    unpacked->ax_bias_mss = (float) packed->ax_bias_mss / 1023.5f - 1.0f;
    unpacked->ay_bias_mss = (float) packed->ax_bias_mss / 1023.5f - 1.0f;
    unpacked->az_bias_mss = (float) packed->ax_bias_mss / 1023.5f - 1.0f;
    unpacked->hx_bias_ut  = (float) packed->hx_bias_ut * 300.0f / 511.5f - 300.0f;
    unpacked->hy_bias_ut  = (float) packed->hy_bias_ut * 300.0f / 511.5f - 300.0f;
    unpacked->hz_bias_ut  = (float) packed->hz_bias_ut * 300.0f / 511.5f - 300.0f;
    unpacked->hx_scale    = (float) packed->hx_scale / 511.5f;
    unpacked->hy_scale    = (float) packed->hy_scale / 511.5f;
    unpacked->hz_scale    = (float) packed->hz_scale / 511.5f;
  }
}

void pack_int_bme280_config(struct int_bme280_config *unpacked, struct msg_int_bme280_config *packed)
{}
void unpack_int_bme280_config(struct msg_int_bme280_config *packed, struct int_bme280_config *unpacked)
{}

void pack_mpu9250_config(struct mpu9250_config *unpacked, struct msg_mpu9250_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->bfs_addr    = unpacked->bfs_addr;
    packed->use_spi     = unpacked->use_spi;
    packed->bus         = (unpacked->bus == 1) ? 0 : 1; 
    packed->addr        = (unpacked->addr == 0x68) ? 0 : 1;
    packed->accel_range = unpacked->accel_range;
    packed->gyro_range  = unpacked->gyro_range;
    packed->srd         = unpacked->srd;
    packed->bandwidth   = unpacked->bandwidth;
    packed->ax_bias_mss = unpacked->ax_bias_mss * 1023.5f + 1023.5f;
    packed->ay_bias_mss = unpacked->ax_bias_mss * 1023.5f + 1023.5f;
    packed->az_bias_mss = unpacked->ax_bias_mss * 1023.5f + 1023.5f;
    packed->hx_bias_ut  = unpacked->hx_bias_ut * 511.5f / 300.0f + 511.5f;
    packed->hy_bias_ut  = unpacked->hy_bias_ut * 511.5f / 300.0f + 511.5f;
    packed->hz_bias_ut  = unpacked->hz_bias_ut * 511.5f / 300.0f + 511.5f;
    packed->hx_scale    = unpacked->hx_scale * 511.5f;
    packed->hy_scale    = unpacked->hy_scale * 511.5f;
    packed->hz_scale    = unpacked->hz_scale * 511.5f;
  }
}

void unpack_mpu9250_config(struct msg_mpu9250_config *packed, struct mpu9250_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->bfs_addr    = packed->bfs_addr;
    unpacked->use_spi     = packed->use_spi;
    unpacked->bus         = (packed->bus == 0) ? 1 : 2; 
    unpacked->addr        = (packed->addr == 0) ? 0x68 : 0x69;
    unpacked->accel_range = packed->accel_range;
    unpacked->gyro_range  = packed->gyro_range;
    unpacked->srd         = packed->srd;
    unpacked->bandwidth   = packed->bandwidth;
    unpacked->ax_bias_mss = (float) packed->ax_bias_mss / 1023.5f - 1.0f;
    unpacked->ay_bias_mss = (float) packed->ax_bias_mss / 1023.5f - 1.0f;
    unpacked->az_bias_mss = (float) packed->ax_bias_mss / 1023.5f - 1.0f;
    unpacked->hx_bias_ut  = (float) packed->hx_bias_ut * 300.0f / 511.5f - 300.0f;
    unpacked->hy_bias_ut  = (float) packed->hy_bias_ut * 300.0f / 511.5f - 300.0f;
    unpacked->hz_bias_ut  = (float) packed->hz_bias_ut * 300.0f / 511.5f - 300.0f;
    unpacked->hx_scale    = (float) packed->hx_scale / 511.5f;
    unpacked->hy_scale    = (float) packed->hy_scale / 511.5f;
    unpacked->hz_scale    = (float) packed->hz_scale / 511.5f;
  }
}

void pack_bme280_config(struct bme280_config *unpacked, struct msg_bme280_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->bfs_addr    = unpacked->bfs_addr;
    packed->use_spi     = unpacked->use_spi;
    packed->bus         = (unpacked->bus == 1) ? 0 : 1; 
    packed->addr        = (unpacked->addr == 0x76) ? 0 : 1;
  }
}

void unpack_bme280_config(struct msg_bme280_config *packed, struct bme280_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->bfs_addr    = packed->bfs_addr;
    unpacked->use_spi     = packed->use_spi;
    unpacked->bus         = (packed->bus == 0) ? 1 : 2; 
    unpacked->addr        = (packed->addr == 0) ? 0x76 : 0x77;
  }
}

void pack_ublox_config(struct ublox_config *unpacked, struct msg_ublox_config *packed)
{
  if ((unpacked) && (packed)) {
      packed->bfs_addr  = unpacked->bfs_addr;
      packed->uart      = unpacked->uart;
    if (unpacked->baud > 460800) {
      packed->baud      = 7;
    } else if (unpacked->baud > 230400) {
      packed->baud      = 6;
    } else if (unpacked->baud > 115200) {
      packed->baud      = 5;
    } else if (unpacked->baud > 57600) {
      packed->baud      = 4;
    } else if (unpacked->baud > 38400) {
      packed->baud      = 3;
    } else if (unpacked->baud > 19200) {
      packed->baud      = 2;
    } else if (unpacked->baud > 9600) {
      packed->baud      = 1;
    } else {
      packed->baud      = 0;
    }
  }
}

void unpack_ublox_config(struct msg_ublox_config *packed, struct ublox_config *unpacked)
{
  if ((unpacked) && (packed)) {
      unpacked->bfs_addr  = packed->bfs_addr;
      unpacked->uart      = packed->uart;
    switch (packed->baud) {
      case 0: {
        unpacked->baud    = 9600;
        break;
      }
      case 1: {
        unpacked->baud    = 19200;
        break;
      }
      case 2: {
        unpacked->baud    = 38400;
        break;
      }
      case 3: {
        unpacked->baud    = 57600;
        break;
      }
      case 4: {
        unpacked->baud    = 115200;
        break;
      }
      case 5: {
        unpacked->baud    = 230400;
        break;
      }
      case 6: {
        unpacked->baud    = 460800;
        break;
      }
      case 7: {
        unpacked->baud    = 921600;
        break;
      }
      default: {
        unpacked->baud    = 115200;
        break;        
      }
    }
  }
}

void pack_sbus_config(struct sbus_config *unpacked, struct msg_sbus_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->bfs_addr = unpacked->bfs_addr;
  }
}

void unpack_sbus_config(struct msg_sbus_config *packed, struct sbus_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->bfs_addr = packed->bfs_addr;
  }
}

void pack_swift_config(struct swift_config *unpacked, struct msg_swift_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->bfs_addr      = unpacked->bfs_addr;
    packed->bus           = (unpacked->bus == 1) ? 0 : 1; 
    packed->static_addr   = unpacked->static_addr;
    packed->diff_addr     = unpacked->diff_addr;
    switch (unpacked->diff_type) {
      case 0: {
        packed->diff_type = 0;
      }
      case 1: {
        packed->diff_type = 1;
      }
      case 4: {
        packed->diff_type = 2;
      }
    }
  }
}

void unpack_swift_config(struct msg_swift_config *packed, struct swift_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->bfs_addr      = packed->bfs_addr;
    unpacked->bus           = (packed->bus == 0) ? 1 : 2; 
    unpacked->static_addr   = packed->static_addr;
    unpacked->diff_addr     = packed->diff_addr;
    switch (packed->diff_type) {
      case 0: {
        unpacked->diff_type = 0;
      }
      case 1: {
        unpacked->diff_type = 1;
      }
      case 2: {
        unpacked->diff_type = 4;
      }
    }
  }
}

void pack_ams5915_config(struct ams5915_config *unpacked, struct msg_ams5915_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->bfs_addr      = unpacked->bfs_addr;
    packed->bus           = (unpacked->bus == 1) ? 0 : 1; 
    packed->addr          = unpacked->addr;
    packed->type          = unpacked->type;
  }
}

void unpack_ams5915_config(struct msg_ams5915_config *packed, struct ams5915_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->bfs_addr      = packed->bfs_addr;
    unpacked->bus           = (packed->bus == 0) ? 1 : 2; 
    unpacked->addr          = packed->addr;
    unpacked->type          = packed->type;
  }
}

void pack_analog_config(struct analog_config *unpacked, struct msg_analog_config *packed)
{
  unsigned int i, temp[8];
  if ((unpacked) && (packed)) {
    packed->bfs_addr      = unpacked->bfs_addr;
    packed->ch            = unpacked->ch; 
    packed->coeff_len     = unpacked->coeff_len;
    for (i = 0; i < ARRAY_SIZE(temp); ++i) {
      memcpy(&temp[i],&unpacked->coeff[i],sizeof(temp[i]));
    }
    packed->coeff0        = temp[0];
    packed->coeff1        = temp[1];
    packed->coeff2        = temp[2];
    packed->coeff3        = temp[3];
    packed->coeff4        = temp[4];
    packed->coeff5        = temp[5];
    packed->coeff6        = temp[6];
    packed->coeff7        = temp[7];
  }
}

void unpack_analog_config(struct msg_analog_config *packed, struct analog_config *unpacked)
{
  unsigned int i, temp[8];
  if ((unpacked) && (packed)) {
    unpacked->bfs_addr    = packed->bfs_addr;
    unpacked->ch          = packed->ch; 
    unpacked->coeff_len   = packed->coeff_len;
    temp[0]               = packed->coeff0;
    temp[1]               = packed->coeff1;
    temp[2]               = packed->coeff2;
    temp[3]               = packed->coeff3;
    temp[4]               = packed->coeff4;
    temp[5]               = packed->coeff5;
    temp[6]               = packed->coeff6;
    temp[7]               = packed->coeff7;
    for (i = 0; i < ARRAY_SIZE(temp); ++i) {
      memcpy(&unpacked->coeff[i],&temp[i],sizeof(unpacked->coeff[i]));
    }
  }
}

void pack_digital_config(struct digital_config *unpacked, struct msg_digital_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->bfs_addr      = unpacked->bfs_addr;
    packed->ch            = unpacked->ch; 
    packed->active_high   = unpacked->active_high;
  }
}

void unpack_digital_config(struct msg_digital_config *packed, struct digital_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->bfs_addr      = packed->bfs_addr;
    unpacked->ch            = packed->ch; 
    unpacked->active_high   = packed->active_high;
  }
}

void pack_voltage_config(struct voltage_config *unpacked, struct msg_voltage_config *packed)
{
  if ((unpacked) && (packed)) {
    packed->bfs_addr      = unpacked->bfs_addr;
    packed->type          = unpacked->type;
  }
}

void unpack_voltage_config(struct msg_voltage_config *packed, struct voltage_config *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->bfs_addr      = packed->bfs_addr;
    unpacked->type          = packed->type;
  }
}

void pack_time_data(struct time_data *unpacked, struct msg_time_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter = unpacked->frame_counter;
    packed->frames        = unpacked->frames;
  }
}

void unpack_time_data(struct msg_time_data *packed, struct time_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter = packed->frame_counter;
    unpacked->frames        = packed->frames;
  }
}

void pack_accel_data(struct accel_data *unpacked, struct msg_accel_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->accel_x_mss     = unpacked->accel_x_mss * 262143.5f / (16.0f * G) + 262143.5f;
    packed->accel_y_mss     = unpacked->accel_y_mss * 262143.5f / (16.0f * G) + 262143.5f;
    packed->accel_z_mss     = unpacked->accel_z_mss * 262143.5f / (16.0f * G) + 262143.5f;
  }
}

void unpack_accel_data(struct msg_accel_data *packed, struct accel_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->accel_x_mss     = (float) packed->accel_x_mss * 16.0f * G / 262143.5f - 16.0f * G;
    unpacked->accel_y_mss     = (float) packed->accel_y_mss * 16.0f * G / 262143.5f - 16.0f * G;
    unpacked->accel_z_mss     = (float) packed->accel_z_mss * 16.0f * G / 262143.5f - 16.0f * G;
  }
}

void pack_gyro_data(struct gyro_data *unpacked, struct msg_gyro_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->gyro_x_rads     = unpacked->gyro_x_rads * 2097151.5f / (2000.0f * D2R);
    packed->gyro_y_rads     = unpacked->gyro_y_rads * 2097151.5f / (2000.0f * D2R);
    packed->gyro_z_rads     = unpacked->gyro_z_rads * 2097151.5f / (2000.0f * D2R);
  }
}

void unpack_gyro_data(struct msg_gyro_data *packed, struct gyro_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->gyro_x_rads     = (float) packed->gyro_x_rads * 2000.0f * D2R / 2097151.5f - 2000.0f * D2R;
    unpacked->gyro_y_rads     = (float) packed->gyro_y_rads * 2000.0f * D2R / 2097151.5f - 2000.0f * D2R;
    unpacked->gyro_z_rads     = (float) packed->gyro_z_rads * 2000.0f * D2R / 2097151.5f - 2000.0f * D2R;
  }
}

void pack_mag_data(struct mag_data *unpacked, struct msg_mag_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->mag_x_ut        = unpacked->mag_x_ut * 131071.5f / 1000.0f + 131071.5f; 
    packed->mag_y_ut        = unpacked->mag_y_ut * 131071.5f / 1000.0f + 131071.5f; 
    packed->mag_z_ut        = unpacked->mag_z_ut * 131071.5f / 1000.0f + 131071.5f; 
  }
}

void unpack_mag_data(struct msg_mag_data *packed, struct mag_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->mag_x_ut     = (float) packed->mag_x_ut * 1000.0f / 131071.5f - 1000.0f;
    unpacked->mag_y_ut     = (float) packed->mag_y_ut * 1000.0f / 131071.5f - 1000.0f;
    unpacked->mag_z_ut     = (float) packed->mag_z_ut * 1000.0f / 131071.5f - 1000.0f;
  }
}

void pack_inceptor_data(struct inceptor_data *unpacked, struct msg_inceptor_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter       = unpacked->frame_counter;
    packed->time_offset_us      = unpacked->time_offset_us;
    packed->lost_frame          = unpacked->lost_frame;
    packed->failsafe_activated  = unpacked->failsafe_activated;
    packed->ch0                 = unpacked->ch[0] * 1023.5f + 1023.5f;
    packed->ch1                 = unpacked->ch[1] * 1023.5f + 1023.5f;
    packed->ch2                 = unpacked->ch[2] * 1023.5f + 1023.5f;
    packed->ch3                 = unpacked->ch[3] * 1023.5f + 1023.5f;
    packed->ch4                 = unpacked->ch[4] * 1023.5f + 1023.5f;
    packed->ch5                 = unpacked->ch[5] * 1023.5f + 1023.5f;
    packed->ch6                 = unpacked->ch[6] * 1023.5f + 1023.5f;
    packed->ch7                 = unpacked->ch[7] * 1023.5f + 1023.5f;
    packed->ch8                 = unpacked->ch[8] * 1023.5f + 1023.5f;
    packed->ch9                 = unpacked->ch[9] * 1023.5f + 1023.5f;
    packed->ch10                = unpacked->ch[10] * 1023.5f + 1023.5f;
    packed->ch11                = unpacked->ch[11] * 1023.5f + 1023.5f;
    packed->ch12                = unpacked->ch[12] * 1023.5f + 1023.5f;
    packed->ch13                = unpacked->ch[13] * 1023.5f + 1023.5f;
    packed->ch14                = unpacked->ch[14] * 1023.5f + 1023.5f;
    packed->ch15                = unpacked->ch[15] * 1023.5f + 1023.5f;
  }
}

void unpack_inceptor_data(struct msg_inceptor_data *packed, struct inceptor_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter       = packed->frame_counter;
    unpacked->time_offset_us      = packed->time_offset_us;
    unpacked->lost_frame          = packed->lost_frame;
    unpacked->failsafe_activated  = packed->failsafe_activated;
    unpacked->ch[0]               = (float) packed->ch0 / 1023.5f - 1.0f;
    unpacked->ch[1]               = (float) packed->ch1 / 1023.5f - 1.0f;
    unpacked->ch[2]               = (float) packed->ch2 / 1023.5f - 1.0f;
    unpacked->ch[3]               = (float) packed->ch3 / 1023.5f - 1.0f;
    unpacked->ch[4]               = (float) packed->ch4 / 1023.5f - 1.0f;
    unpacked->ch[5]               = (float) packed->ch5 / 1023.5f - 1.0f;
    unpacked->ch[6]               = (float) packed->ch6 / 1023.5f - 1.0f;
    unpacked->ch[7]               = (float) packed->ch7 / 1023.5f - 1.0f;
    unpacked->ch[8]               = (float) packed->ch8 / 1023.5f - 1.0f;
    unpacked->ch[9]               = (float) packed->ch9 / 1023.5f - 1.0f;
    unpacked->ch[10]              = (float) packed->ch10 / 1023.5f - 1.0f;
    unpacked->ch[11]              = (float) packed->ch11 / 1023.5f - 1.0f;
    unpacked->ch[12]              = (float) packed->ch12 / 1023.5f - 1.0f;
    unpacked->ch[13]              = (float) packed->ch13 / 1023.5f - 1.0f;
    unpacked->ch[14]              = (float) packed->ch14 / 1023.5f - 1.0f;
    unpacked->ch[15]              = (float) packed->ch15 / 1023.5f - 1.0f;
  }
}

void pack_temperature_data(struct temperature_data *unpacked, struct msg_temperature_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->temp_c          = unpacked->temp_c * 16383.0f / 120.0f + 5461.0f;
  }
}

void unpack_temperature_data(struct msg_temperature_data *packed, struct temperature_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->temp_c          = (float) packed->temp_c * 120.0f / 16383.0f - 40.0f;
  }
}

void pack_gnss_data(struct gnss_data *unpacked, struct msg_gnss_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->fix             = unpacked->fix;
    packed->num_sv          = unpacked->num_sv;
    packed->tow_ms          = unpacked->tow_ms;
    packed->lat_rad         = unpacked->lat_rad * 2147483647.5 / D_PI + 2147483647.5;
    packed->lon_rad         = unpacked->lon_rad * 2147483647.5 / D_PI + 2147483647.5;
    packed->hmsl_m          = unpacked->hmsl_m * 1118.48105f + 11184810.5f;
    packed->vel_north_ms    = unpacked->vel_north_ms * 1048.575f + 524287.5f;
    packed->vel_east_ms     = unpacked->vel_east_ms * 1048.575f + 524287.5f;
    packed->vel_down_ms     = unpacked->vel_down_ms * 1310.715f + 131071.5f;
    packed->horiz_acc_m     = unpacked->horiz_acc_m * 1310.71f;
    packed->vert_acc_m      = unpacked->vert_acc_m * 1310.71f;
    packed->speed_acc_ms    = unpacked->speed_acc_ms * 1310.71f;
    packed->pdop            = unpacked->pdop * 163.83f;
  }
}

void unpack_gnss_data(struct msg_gnss_data *packed, struct gnss_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->fix             = packed->fix;
    unpacked->num_sv          = packed->num_sv;
    unpacked->tow_ms          = packed->tow_ms;
    unpacked->lat_rad         = (double) packed->lat_rad * D_PI / 2147483647.5 - D_PI;
    unpacked->lon_rad         = (double) packed->lon_rad * D_PI / 2147483647.5 - D_PI;
    unpacked->hmsl_m          = (float) packed->hmsl_m * 60000.0f / 67108863.0f - 10000.0f;
    unpacked->vel_north_ms    = (float) packed->vel_north_ms * 500.0f / 524287.5f - 500.0f;
    unpacked->vel_east_ms     = (float) packed->vel_east_ms * 500.0f / 524287.5f - 500.0f;
    unpacked->vel_down_ms     = (float) packed->vel_down_ms * 100.0f / 131071.5f - 100.0f;
    unpacked->horiz_acc_m     = (float) packed->horiz_acc_m / 1310.71f;
    unpacked->vert_acc_m      = (float) packed->vert_acc_m / 1310.71f;
    unpacked->speed_acc_ms    = (float) packed->speed_acc_ms / 1310.71f;
    unpacked->pdop            = (float) packed->pdop / 163.83f;
  }
}

void pack_static_press_data(struct static_press_data *unpacked, struct msg_static_press_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->press_pa        = unpacked->press_pa * 4194303.0f / 80000.0f - 1572863.625f;
  }
}

void unpack_static_press_data(struct msg_static_press_data *packed, struct static_press_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->press_pa        = (float) packed->press_pa * 80000.0f / 4194303.0f + 30000.0f; 
  }
}

void pack_diff_press_data(struct diff_press_data *unpacked, struct msg_diff_press_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->press_pa        = unpacked->press_pa * 2097151.5f / 20000.0f + 2097151.5f;
  }
}

void unpack_diff_press_data(struct msg_diff_press_data *packed, struct diff_press_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->press_pa        = (float) packed->press_pa * 20000.0f / 2097151.5f - 20000.0f;
  }
}

void pack_analog_data(struct analog_data *unpacked, struct msg_analog_data *packed)
{
  unsigned int temp;
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->voltage         = unpacked->voltage * 8191.0f / 3.3f;
    memcpy(&temp,&unpacked->cal_value,sizeof(temp));
    packed->cal_value       = temp;
  }
}

void unpack_analog_data(struct msg_analog_data *packed, struct analog_data *unpacked)
{
  unsigned int temp;
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->voltage         = (float) packed->voltage * 3.3f / 8191.0f;
    temp                      = packed->cal_value;
    memcpy(&unpacked->cal_value,&temp,sizeof(unpacked->cal_value));
  }
}

void pack_digital_data(struct digital_data *unpacked, struct msg_digital_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->digout          = unpacked->digout;
  }
}

void unpack_digital_data(struct msg_digital_data *packed, struct digital_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->digout          = packed->digout;
  }
}

void pack_voltage_data(struct voltage_data *unpacked, struct msg_voltage_data *packed)
{
  if ((unpacked) && (packed)) {
    packed->frame_counter   = unpacked->frame_counter;
    packed->time_offset_us  = unpacked->time_offset_us;
    packed->voltage         = unpacked->voltage * 8191.0f / 36.0f;
  }
}

void unpack_voltage_data(struct msg_voltage_data *packed, struct voltage_data *unpacked)
{
  if ((unpacked) && (packed)) {
    unpacked->frame_counter   = packed->frame_counter;
    unpacked->time_offset_us  = packed->time_offset_us;
    unpacked->voltage         = (float) packed->voltage * 36.0f / 8191.0f;
  }
}
