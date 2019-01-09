//
// FILE: FGFS.cxx
// DESCRIPTION: aquire live sensor data from an running copy of Flightgear
//

#include <string>
using std::string;

#include <stdlib.h>		// drand48()
#include <sys/ioctl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
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
static float press_pa = 0.0;

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
static ElementPtr press_node;


static ElementPtr cmdAilL_node;
static ElementPtr cmdAilR_node;
static ElementPtr cmdElev_node;
static ElementPtr cmdRud_node;
static ElementPtr cmdFlapL_node;
static ElementPtr cmdFlapR_node;

static ElementPtr cmdTE1L_node;
static ElementPtr cmdTE1R_node;
static ElementPtr cmdTE2L_node;
static ElementPtr cmdTE2R_node;
static ElementPtr cmdTE3L_node;
static ElementPtr cmdTE3R_node;
static ElementPtr cmdTE4L_node;
static ElementPtr cmdTE4R_node;
static ElementPtr cmdTE5L_node;
static ElementPtr cmdTE5R_node;
static ElementPtr cmdLEL_node;
static ElementPtr cmdLER_node;

static ElementPtr cmdMotor_node;
static ElementPtr cmdGear_node;


static const float D2R = M_PI / 180.0;
static const float SG_METER_TO_FEET = 1.0 / 0.3048;
static const float KTS_TO_MPS = 0.514444;

Vector3f mag_ned;
Vector3f mag_body;
Quaternionf q_N2B;
Matrix3f C_N2B;

static bool fgfs_interface_active = false;
string modelName;

// Todo: make flightgear ports and host configurable
bool fgfs_init( const rapidjson::Value& Config ) {
  if ( Config.HasMember("FlightGear") ) {
    const rapidjson::Value& FgConfig = Config["FlightGear"];
    if ( FgConfig.HasMember("Model") ) {
      modelName = FgConfig["Model"].GetString();
    }
    fgfs_imu_init();
    fgfs_gps_init();
    fgfs_airdata_init();
    fgfs_act_init();

    std::cout << "FlightGear interface initialized: " << modelName << std::endl;
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

  if (modelName == "mAEWing2") {
    cmdTE1L_node = deftree.getElement("/Control/cmdTE1L_rad");
    cmdTE1R_node = deftree.getElement("/Control/cmdTE1R_rad");
    cmdTE2L_node = deftree.getElement("/Control/cmdTE2L_rad");
    cmdTE2R_node = deftree.getElement("/Control/cmdTE2R_rad");
    cmdTE3L_node = deftree.getElement("/Control/cmdTE3L_rad");
    cmdTE3R_node = deftree.getElement("/Control/cmdTE3R_rad");
    cmdTE4L_node = deftree.getElement("/Control/cmdTE4L_rad");
    cmdTE4R_node = deftree.getElement("/Control/cmdTE4R_rad");
    cmdTE5L_node = deftree.getElement("/Control/cmdTE5L_rad");
    cmdTE5R_node = deftree.getElement("/Control/cmdTE5R_rad");
    cmdLEL_node = deftree.getElement("/Control/cmdLEL_rad");
    cmdLER_node = deftree.getElement("/Control/cmdLER_rad");
    cmdMotor_node = deftree.getElement("/Control/cmdMotor_nd");
    cmdGear_node = deftree.getElement("/Control/cmdGear_nd");
  } else if ((modelName == "UltraStick25e") || (modelName == "UltraStick120")) {
    cmdAilL_node = deftree.getElement("/Control/cmdAilL_rad");
    cmdAilR_node = deftree.getElement("/Control/cmdAilR_rad");
    cmdElev_node = deftree.getElement("/Control/cmdElev_rad");
    cmdRud_node = deftree.getElement("/Control/cmdRud_rad");
    cmdFlapL_node = deftree.getElement("/Control/cmdFlapL_rad");
    cmdFlapR_node = deftree.getElement("/Control/cmdFlapR_rad");
    cmdMotor_node = deftree.getElement("/Control/cmdMotor_nd");
  } else {
    std::cout << "Flight Gear 'Model' not understood" << std::endl;
  }

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
  press_node = deftree.getElement("/Sensors/Pitot/Static/Pressure_Pa");
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
  while ( (result = sock_imu.recv(packet_buf, fgfs_imu_size, 0)) == fgfs_imu_size ) {
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
    const double inhg2pa = 3386.38866667;
    press_pa = pressure * inhg2pa;
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
  press_node->setFloat( press_pa );
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

  if (modelName == "mAEWing2") {
    float cmdTE1L_rad = cmdTE1L_node->getFloat();
    *(float *)buf = cmdTE1L_rad; buf += 4;

    float cmdTE1R_rad = cmdTE1R_node->getFloat();
    *(float *)buf = cmdTE1R_rad; buf += 4;

    float cmdTE2L_rad = cmdTE2L_node->getFloat();
    *(float *)buf = cmdTE2L_rad; buf += 4;

    float cmdTE2R_rad = cmdTE2R_node->getFloat();
    *(float *)buf = cmdTE2R_rad; buf += 4;

    float cmdTE3L_rad = cmdTE3L_node->getFloat();
    *(float *)buf = cmdTE3L_rad; buf += 4;

    float cmdTE3R_rad = cmdTE3R_node->getFloat();
    *(float *)buf = cmdTE3R_rad; buf += 4;

    float cmdTE4L_rad = cmdTE4L_node->getFloat();
    *(float *)buf = cmdTE4L_rad; buf += 4;

    float cmdTE4R_rad = cmdTE4R_node->getFloat();
    *(float *)buf = cmdTE4R_rad; buf += 4;

    float cmdTE5L_rad = cmdTE5L_node->getFloat();
    *(float *)buf = cmdTE5L_rad; buf += 4;

    float cmdTE5R_rad = cmdTE5R_node->getFloat();
    *(float *)buf = cmdTE5R_rad; buf += 4;

    float cmdLEL_rad = cmdLEL_node->getFloat();
    *(float *)buf = cmdLEL_rad; buf += 4;

    float cmdLER_rad = cmdLER_node->getFloat();
    *(float *)buf = cmdLER_rad; buf += 4;

    float cmdMotor_nd = cmdMotor_node->getFloat();
    *(float *)buf = cmdMotor_nd; buf += 4;

    float cmdGear_nd = cmdGear_node->getFloat();
    *(float *)buf = cmdGear_nd; buf += 4;

    float ch15 = 0.0;
    *(float *)buf = ch15; buf += 4;

    float ch16 = 0.0;
    *(float *)buf = ch16; buf += 4;

    float ch17 = 0.0;
    *(float *)buf = ch17; buf += 4;

  } else if ((modelName == "UltraStick25e") || (modelName == "UltraStick120")) {

    float cmdAilL_rad = cmdAilL_node->getFloat();
    *(float *)buf = cmdAilL_rad; buf += 4;

    float cmdAilR_rad = cmdAilR_node->getFloat();
    *(float *)buf = cmdAilR_rad; buf += 4;

    float cmdElev_rad = cmdElev_node->getFloat();
    *(float *)buf = cmdElev_rad; buf += 4;

    float cmdRud_rad = cmdRud_node->getFloat();
    *(float *)buf = cmdRud_rad; buf += 4;

    float cmdFlapL_rad = cmdFlapL_node->getFloat();
    *(float *)buf = cmdFlapL_rad; buf += 4;

    float cmdFlapR_rad = cmdFlapR_node->getFloat();
    *(float *)buf = cmdFlapR_rad; buf += 4;

    float cmdMotor_nd = cmdMotor_node->getFloat();
    *(float *)buf = cmdMotor_nd; buf += 4;

    float ch8 = 0.0;
    *(float *)buf = ch8; buf += 4;

    float ch9 = 0.0;
    *(float *)buf = ch9; buf += 4;

    float ch10 = 0.0;
    *(float *)buf = ch10; buf += 4;

    float ch11 = 0.0;
    *(float *)buf = ch11; buf += 4;

    float ch12 = 0.0;
    *(float *)buf = ch12; buf += 4;

    float ch13 = 0.0;
    *(float *)buf = ch13; buf += 4;

    float ch14 = 0.0;
    *(float *)buf = ch14; buf += 4;

    float ch15 = 0.0;
    *(float *)buf = ch15; buf += 4;

    float ch16 = 0.0;
    *(float *)buf = ch16; buf += 4;

    float ch17 = 0.0;
    *(float *)buf = ch17; buf += 4;
  } else {
    std::cout << "Flight Gear 'Model' not understood" << std::endl;
  }

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
