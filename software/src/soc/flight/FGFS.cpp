//
// FILE: FGFS.cxx
// DESCRIPTION: aquire live sensor data from an running copy of Flightgear
//

#include <string>
using std::string;

#include <stdlib.h>		// drand48()
#include <sys/ioctl.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;
#include <iostream>
using std::cout;
using std::endl;

#include "definition-tree2.h"
#include "coremag.h"
#include "nav_functions_float.hxx"
#include "netSocket.h"
#include "FGFS.h"

static netSocket sock_imu;
static netSocket sock_gps;
static netSocket sock_act;

static int port_imu = 6500;
static int port_gps = 6501;
static int port_act = 6503;
static string host_act = "192.168.7.1";

// local static values
static double imu_time = 0.0;
static float p = 0.0;
static float q = 0.0;
static float r = 0.0;
static float ax = 0.0;
static float ay = 0.0;
static float az = 0.0;
static float hx = 0.0;
static float hy = 0.0;
static float hz = 0.0;

static double gps_time = 0.0;
static double lat = 0.0;
static double lon = 0.0;
static double alt = 0.0;
static double vn = 0.0;
static double ve = 0.0;
static double vd = 0.0;
static uint8_t sats = 0;
static uint8_t fix = 0;

static float ias_kt = 0.0;

static float cmd_ail = 0.0;
static float cmd_ele = 0.0;
static float cmd_thr = 0.0;
static float cmd_rud = 0.0;
static float cmd_ch5 = 0.0;
static float cmd_ch6 = 0.0;
static float cmd_ch7 = 0.0;
static float cmd_ch8 = 0.0;
static float tgt_roll = 0.0;
static float tgt_pitch = 0.0;
static float tgt_yaw = 0.0;
static float tgt_climb = 0.0;
static float tgt_alt = 0.0;
static float tgt_kts = 0.0;
static float tgt_offset = 0.0;
static float wp_dist = 0.0;
static float wp_eta = 0.0;

// pointers to def tree
static ElementPtr p_node;
static ElementPtr q_node;
static ElementPtr r_node;
static ElementPtr ax_node;
static ElementPtr ay_node;
static ElementPtr az_node;
static ElementPtr hx_node;
static ElementPtr hy_node;
static ElementPtr hz_node;

static ElementPtr lat_node;
static ElementPtr lon_node;
static ElementPtr alt_node;
static ElementPtr vn_node;
static ElementPtr ve_node;
static ElementPtr vd_node;
static ElementPtr sats_node;
static ElementPtr fix_node;

static ElementPtr ias_node;

static ElementPtr cmd_left_ail_node;
static ElementPtr cmd_right_ail_node;
static ElementPtr cmd_ele_node;
static ElementPtr cmd_thr_node;
static ElementPtr cmd_rud_node;

static const float D2R = M_PI / 180.0;
static const float SG_METER_TO_FEET = 1.0 / 0.3048;
static const float KTS_TO_MPS = 0.514444;

Vector3f mag_ned;
Vector3f mag_body;
Quaternionf q_N2B;
Matrix3f C_N2B;

static bool fgfs_interface_active = false;

// Todo: make flightgear ports and host configurable
bool fgfs_init( const rapidjson::Value& Config ) {
  if ( Config.HasMember("FlightGear") ) {
    fgfs_imu_init();
    fgfs_gps_init();
    fgfs_airdata_init();
    fgfs_act_init();

    printf("FlightGear interface initialized\n");
    fgfs_interface_active = true;

    return true;
  } else {
    return false;
  }  
}

bool fgfs_imu_init() {
  printf("fgfs_imu_init()\n");
    
  p_node = deftree.getElement("/Sensors/Fmu/Mpu9250/GyroX_rads");
  q_node = deftree.getElement("/Sensors/Fmu/Mpu9250/GyroY_rads");
  r_node = deftree.getElement("/Sensors/Fmu/Mpu9250/GyroZ_rads");
  ax_node = deftree.getElement("/Sensors/Fmu/Mpu9250/AccelX_mss");
  ay_node = deftree.getElement("/Sensors/Fmu/Mpu9250/AccelY_mss");
  az_node = deftree.getElement("/Sensors/Fmu/Mpu9250/AccelZ_mss");
  hx_node = deftree.getElement("/Sensors/Fmu/Mpu9250/MagX_uT");
  hy_node = deftree.getElement("/Sensors/Fmu/Mpu9250/MagY_uT");
  hz_node = deftree.getElement("/Sensors/Fmu/Mpu9250/MagZ_uT");

  // open a UDP socket
  if ( ! sock_imu.open( false ) ) {
    printf("Error opening imu input socket\n");
    return false;
  }

  // bind ...
  if ( sock_imu.bind( "", port_imu ) == -1 ) {
    printf("error binding to port %d\n", port_imu );
    return false;
  }

  // don't block waiting for input
  sock_imu.setBlocking( false );
    
  return true;
}


bool fgfs_gps_init() {
  printf("fgfs_gps_init()\n");
  // bind def tree pointers
  lon_node = deftree.getElement("/Sensors/uBlox/Longitude_rad");
  lat_node = deftree.getElement("/Sensors/uBlox/Latitude_rad");
  alt_node = deftree.getElement("/Sensors/uBlox/Altitude_m");
  vn_node = deftree.getElement("/Sensors/uBlox/NorthVelocity_ms");
  ve_node = deftree.getElement("/Sensors/uBlox/EastVelocity_ms");
  vd_node = deftree.getElement("/Sensors/uBlox/DownVelocity_ms");
  sats_node = deftree.getElement("/Sensors/uBlox/NumberSatellites");
  fix_node = deftree.getElement("/Sensors/uBlox/Fix");

  // some initial value
  mag_ned << 0.5, 0.01, -0.9;
  mag_ned.normalize();
    
  // open a UDP socket
  if ( ! sock_gps.open( false ) ) {
    printf("Error opening imu input socket\n");
    return false;
  }

  // bind ...
  if ( sock_gps.bind( "", port_gps ) == -1 ) {
    printf("error binding to port %d\n", port_gps );
    return false;
  }

  // don't block waiting for input
  sock_gps.setBlocking( false );

  return true;
}


bool fgfs_act_init() {
  printf("fgfs_act_init()\n");

  cmd_left_ail_node = deftree.getElement("/Control/cmdAilL_rad");
  cmd_right_ail_node = deftree.getElement("/Control/cmdAilR_rad");
  cmd_ele_node = deftree.getElement("/Control/cmdElev_rad");
  cmd_thr_node = deftree.getElement("/Control/cmdMotor_nd");
  cmd_rud_node = deftree.getElement("/Control/cmdRud_rad");

  // open a UDP socket
  if ( ! sock_act.open( false ) ) {
    printf("Error opening imu input socket\n");
    return false;
  }

  // connect ...
  if ( sock_act.connect( host_act.c_str(), port_act ) == -1 ) {
    printf("error connecting to %s:%d\n", host_act.c_str(), port_act);
    return false;
  }

  // don't block
  sock_act.setBlocking( false );

  return true;
}


bool fgfs_airdata_init() {
  printf("fgfs_airdata_init()\n");
  ias_node = deftree.getElement("/Sensor-Processing/vIAS_ms");
  return true;
}

// swap big/little endian bytes
static void my_swap( uint8_t *buf, int index, int count )
{
  int i;
  uint8_t tmp;
  for ( i = 0; i < count / 2; ++i ) {
    tmp = buf[index+i];
    buf[index+i] = buf[index+count-i-1];
    buf[index+count-i-1] = tmp;
  }
}


bool fgfs_imu_update() {
  const int fgfs_imu_size = 52;
  uint8_t packet_buf[fgfs_imu_size];

  bool fresh_data = false;

  int result;
  if ( (result = sock_imu.recv(packet_buf, fgfs_imu_size, 0))
       == fgfs_imu_size )
    {
      fresh_data = true;

      if ( ulIsLittleEndian ) {
        my_swap( packet_buf, 0, 8 );
        my_swap( packet_buf, 8, 4 );
        my_swap( packet_buf, 12, 4 );
        my_swap( packet_buf, 16, 4 );
        my_swap( packet_buf, 20, 4 );
        my_swap( packet_buf, 24, 4 );
        my_swap( packet_buf, 28, 4 );
        my_swap( packet_buf, 32, 4 );
        my_swap( packet_buf, 36, 4 );
        my_swap( packet_buf, 40, 4 );
        my_swap( packet_buf, 44, 4 );
        my_swap( packet_buf, 48, 4 );
      }

      uint8_t *buf = packet_buf;
      imu_time = *(double *)buf; buf += 8;
      p = *(float *)buf; buf += 4;
      q = *(float *)buf; buf += 4;
      r = *(float *)buf; buf += 4;
      ax = *(float *)buf; buf += 4;
      ay = *(float *)buf; buf += 4;
      az = *(float *)buf; buf += 4;
      ias_kt = *(float *)buf; buf += 4;
      float pressure = *(float *)buf; buf += 4;
      float roll_truth = *(float *)buf; buf += 4;
      float pitch_truth = *(float *)buf; buf += 4;
      float yaw_truth = *(float *)buf; buf += 4;

      // generate fake magnetometer readings
      q_N2B = eul2quat(roll_truth * D2R, pitch_truth * D2R, yaw_truth * D2R);
      // rotate ideal mag vector into body frame (then normalized)
      mag_body = q_N2B.inverse() * mag_ned;
      mag_body.normalize();
      // cout << "mag vector (body): " << mag_body(0) << " " << mag_body(1) << " " << mag_body(2) << endl;

      //airdata_node.setDouble( "timestamp", cur_time );
      //airdata_node.setDouble( "airspeed_kt", airspeed );
      const double inhg2mbar = 33.8638866667;
      //airdata_node.setDouble( "pressure_mbar", pressure * inhg2mbar );

      // fake volt/amp values here for no better place to do it
      // fixme: static double last_time = cur_time;
      static double mah = 0.0;
      //double thr = act_node.getDouble("throttle");
      //power_node.setDouble("main_vcc", 16.0 - thr);
      //int cells = config_specs_node.getLong("battery_cells");
      //if ( cells < 1 ) { cells = 4; }
      //power_node.setDouble("cell_vcc", (16.0 - thr) / cells);
      //power_node.setDouble("main_amps", thr * 12.0);
      //double dt = cur_time - last_time;
      //mah += thr*75.0 * (1000.0/3600.0) * dt;
      // fixme: last_time = cur_time;
      //power_node.setDouble( "total_mah", mah );
    }

  p_node->setFloat(p);
  q_node->setFloat(q);
  r_node->setFloat(r);
  ax_node->setFloat(ax);
  ay_node->setFloat(ay);
  az_node->setFloat(az);
  hx_node->setFloat(mag_body(0));
  hy_node->setFloat(mag_body(1));
  hz_node->setFloat(mag_body(2));
  // cout << "imu: " << *p_node << " " << *q_node << " " << *r_node << " " << *ax_node << " " << *ay_node << " " << *az_node << " " << *hx_node << " " << *hy_node << " " << *hz_node << endl;

  return fresh_data;
}

bool fgfs_gps_update() {
  const int fgfs_gps_size = 40;
  uint8_t packet_buf[fgfs_gps_size];

  bool fresh_data = false;

  int result;
  while ( (result = sock_gps.recv(packet_buf, fgfs_gps_size, 0)) == fgfs_gps_size ) {
    fresh_data = true;
    fix = 1;

    if ( ulIsLittleEndian ) {
      my_swap( packet_buf, 0, 8 );
      my_swap( packet_buf, 8, 8 );
      my_swap( packet_buf, 16, 8 );
      my_swap( packet_buf, 24, 4 );
      my_swap( packet_buf, 28, 4 );
      my_swap( packet_buf, 32, 4 );
      my_swap( packet_buf, 36, 4 );
    }

    uint8_t *buf = packet_buf;
    gps_time = *(double *)buf; buf += 8;
    lat = *(double *)buf; buf += 8;
    lon = *(double *)buf; buf += 8;
    alt = *(float *)buf; buf += 4;
    vn = *(float *)buf; buf += 4;
    ve = *(float *)buf; buf += 4;
    vd = *(float *)buf; buf += 4;

    // cout << gps_time << " " << lat << " " << lon << " " << alt << endl;
        
    // compute ideal magnetic vector in ned frame
    long int jd = now_to_julian_days();
    double field[6];
    calc_magvar( lat*D2R, lon*D2R, alt / 1000.0, jd, field );
    mag_ned(0) = field[3];
    mag_ned(1) = field[4];
    mag_ned(2) = field[5];
    mag_ned.normalize();
    // cout << "mag vector (ned): " << mag_ned(0) << " " << mag_ned(1) << " " << mag_ned(2) << endl;

  }

  // write values even if data isn't fresh (to overwrite real sensor data)
  lon_node->setDouble(lon * D2R);
  lat_node->setDouble(lat * D2R);
  alt_node->setFloat(alt);
  vn_node->setFloat(vn);
  ve_node->setFloat(ve);
  vd_node->setFloat(vd);
  sats_node->setInt(8);
  fix_node->setInt(fix);

  // cout << "gps: " << *lon_node << " " << *lat_node << " " << *alt_node << " " << *vn_node << " " << *ve_node << " " << *vd_node << endl;
    
  return fresh_data;
}


bool fgfs_act_update() {
  // additional autopilot target nodes (note this is a hack, but we
  // are sending data back to FG in this module so it makes some
  // sense to include autopilot targets.)

  const int fgfs_act_size = 76;
  uint8_t packet_buf[fgfs_act_size];
  uint8_t *buf = packet_buf;

  double time = 0.0;
  *(double *)buf = time; buf += 8;

  float ail = (cmd_left_ail_node->getFloat() - cmd_right_ail_node->getFloat());
  *(float *)buf = ail; buf += 4;

  float ele = cmd_ele_node->getFloat() * 2.0;
  *(float *)buf = ele; buf += 4;

  float thr = cmd_thr_node->getFloat();
  *(float *)buf = thr; buf += 4;

  float rud = cmd_rud_node->getFloat() * -2.0;
  *(float *)buf = rud; buf += 4;

  float ch5 = 0.0;
  *(float *)buf = ch5; buf += 4;

  float ch6 = 0.0;
  *(float *)buf = ch6; buf += 4;

  float ch7 = 0.0;
  *(float *)buf = ch7; buf += 4;

  float ch8 = 0.0;
  *(float *)buf = ch8; buf += 4;

  float bank = 0.0;
  *(float *)buf = bank; buf += 4;

  float pitch = 0.0;
  *(float *)buf = pitch; buf += 4;

  float target_track_offset = 0.0;
  if ( target_track_offset < -180 ) { target_track_offset += 360.0; }
  if ( target_track_offset > 180 ) { target_track_offset -= 360.0; }
  float hdg = target_track_offset * 100 + 36000.0;
  *(float *)buf = hdg; buf += 4;

  // FIXME: no longer used so wasted 4 bytes ...
  float climb = 0.0;
  *(float *)buf = climb; buf += 4;

  float alt_agl_ft = 0.0;
  float ground_m = 0.0;
  float alt_msl_ft = (ground_m * SG_METER_TO_FEET + alt_agl_ft) * 100.0;
  *(float *)buf = alt_msl_ft; buf += 4;

  float speed = 0.0;
  *(float *)buf = speed; buf += 4;

  float track_offset = 0.0;
  if ( track_offset < -180 ) { track_offset += 360.0; }
  if ( track_offset > 180 ) { track_offset -= 360.0; }
  float offset = track_offset * 100 + 36000.0;
  *(float *)buf = offset; buf += 4;

  float dist = 0.0;
  *(float *)buf = dist; buf += 4;

  float eta = 0.0;
  *(float *)buf = eta; buf += 4;

  if ( ulIsLittleEndian ) {
    my_swap( packet_buf, 0, 8 );
    my_swap( packet_buf, 8, 4 );
    my_swap( packet_buf, 12, 4 );
    my_swap( packet_buf, 16, 4 );
    my_swap( packet_buf, 20, 4 );
    my_swap( packet_buf, 24, 4 );
    my_swap( packet_buf, 28, 4 );
    my_swap( packet_buf, 32, 4 );
    my_swap( packet_buf, 36, 4 );
    my_swap( packet_buf, 40, 4 );
    my_swap( packet_buf, 44, 4 );
    my_swap( packet_buf, 48, 4 );
    my_swap( packet_buf, 52, 4 );
    my_swap( packet_buf, 56, 4 );
    my_swap( packet_buf, 60, 4 );
    my_swap( packet_buf, 64, 4 );
    my_swap( packet_buf, 68, 4 );
    my_swap( packet_buf, 72, 4 );
  }

  int result = sock_act.send( packet_buf, fgfs_act_size, 0 );
  if ( result != fgfs_act_size ) {
    return false;
  }

  return true;
}

bool fgfs_airdata_update() {
  ias_node->setFloat( ias_kt * KTS_TO_MPS );
  return true;
}

void fgfs_imu_close() {
  sock_imu.close();
}

void fgfs_gps_close() {
  sock_gps.close();
}

void fgfs_act_close() {
  sock_act.close();
}
