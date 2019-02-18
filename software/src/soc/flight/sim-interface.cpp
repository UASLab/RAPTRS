//
// FILE: sim-interface.cpp
// DESCRIPTION: aquire live sensor data from an running copy of JSBSim
//

#include <stdlib.h>
#include <iostream>
#include <string>

#include "definition-tree2.h"
#include "netSocket.h"
#include "sim-interface.h"

//
static bool sim_interface_active = false;
static std::string modelName;
static int port_imu = 59011;
static bool run_imu = 0;
static int port_gps = 59012;
static bool run_gps = 0;
static int port_pitot = 59013;
static bool run_pitot = 0;
static int port_five1 = 59014;
static bool run_five1 = 0;
static int port_five2 = 59015;
static bool run_five2 = 0;
static int port_cmd = 59051;
static std::string host_cmd = "192.168.7.1";
static bool run_cmd = 0;

// Sim Init
bool sim_init( const rapidjson::Value& Config ) {
  if ( Config.HasMember("JSBSim") ) {
    const rapidjson::Value& JsbConfig = Config["JSBSim"];

    if ( JsbConfig.HasMember("Model") ) {
      modelName = JsbConfig["Model"].GetString();
    }

    if ( JsbConfig.HasMember("ImuPort") ) {
      port_imu = JsbConfig["ImuPort"].GetInt();
      sim_imu_init();
      run_imu = 1;
      std::cout << "IMU interface initialized on Port: " << port_imu << std::endl;
    }

    if ( JsbConfig.HasMember("GpsPort") ) {
      port_gps = JsbConfig["GpsPort"].GetInt();
      sim_gps_init();
      run_gps = 1;
      std::cout << "Gps interface initialized on Port: " << port_gps << std::endl;
    }

    if ( JsbConfig.HasMember("PitotPort") ) {
      port_pitot = JsbConfig["PitotPort"].GetInt();
      sim_pitot_init();
      run_pitot = 1;
      std::cout << "Pitot interface initialized on Port: " << port_pitot << std::endl;
    }

    if ( JsbConfig.HasMember("5hole1Port") ) {
      port_five1 = JsbConfig["5hole1Port"].GetInt();
      sim_5hole1_init();
      run_five1 = 1;
      std::cout << "Five Hole [Method1] interface initialized on Port: " << port_five1 << std::endl;
    }

    if ( JsbConfig.HasMember("5hole2Port") ) {
      port_five2 = JsbConfig["5hole2Port"].GetInt();
      sim_5hole2_init();
      run_five2 = 1;
      std::cout << "Five Hole [Method2] interface initialized on Port: " << port_five2 << std::endl;
    }

    // Init output message
    if (( JsbConfig.HasMember("CmdPort")) & ( JsbConfig.HasMember("CmdHost"))) {
      port_cmd = JsbConfig["CmdPort"].GetInt();
      host_cmd = JsbConfig["CmdHost"].GetString();
      sim_cmd_init();
      run_cmd = 1;
      std::cout << "Output Cmd interface initialized on Port: " << host_cmd << ":" << port_cmd << std::endl;
    }

    // if (JsbConfig.HasMember("CmdList")) {
    //   for (size_t i=0; i < JsbConfig["CmdList"].Size(); i++) {
    //     const rapidjson::Value& Input = JsbConfig["CmdList"][i];
    //     CmdKeys_.push_back(Input.GetString());
    //
    //     ElementPtr ele = deftree.getElement(CmdKeys_.back());
    //     if ( ele ) {
    //       cmd_nodes.push_back(ele);
    //     } else {
    //       throw std::runtime_error(std::string("ERROR ")+"CmdList"+std::string(": Input ")+CmdKeys_.back()+std::string(" not found in global data."));
    //     }
    //   }
    // }

    std::cout << "JSBSim interface initialized: " << modelName << std::endl;
    sim_interface_active = true;

    return true;
  } else {
    return false;
  }
}

// Sim Update
bool sim_sensor_update( ) {
  if ( run_imu ) { sim_imu_update(); }
  if ( run_gps ) { sim_gps_update(); }
  if ( run_pitot ) { sim_pitot_update(); }
  if ( run_five1 ) { sim_5hole1_update(); }
  if ( run_five2 ) { sim_5hole2_update(); }
  return true;
}


// IMU
static netSocket sock_imu;

static float imuTime_s = 0.0;
static double imuTime_us = 0.0;
static float p_rps = 0.0;
static float q_rps = 0.0;
static float r_rps = 0.0;
static float ax_mps2 = 0.0;
static float ay_mps2 = 0.0;
static float az_mps2 = 0.0;
static float hx_uT = 0.0;
static float hy_uT = 0.0;
static float hz_uT = 0.0;

static ElementPtr imuTime_node;
static ElementPtr p_node;
static ElementPtr q_node;
static ElementPtr r_node;
static ElementPtr ax_node;
static ElementPtr ay_node;
static ElementPtr az_node;
static ElementPtr hx_node;
static ElementPtr hy_node;
static ElementPtr hz_node;

bool sim_imu_init() {
  printf("sim_imu_init()\n");

  imuTime_node = deftree.getElement("/Sensors/Fmu/Time_us");
  ax_node = deftree.getElement("/Sensors/Fmu/Mpu9250/AccelX_mss");
  ay_node = deftree.getElement("/Sensors/Fmu/Mpu9250/AccelY_mss");
  az_node = deftree.getElement("/Sensors/Fmu/Mpu9250/AccelZ_mss");
  p_node = deftree.getElement("/Sensors/Fmu/Mpu9250/GyroX_rads");
  q_node = deftree.getElement("/Sensors/Fmu/Mpu9250/GyroY_rads");
  r_node = deftree.getElement("/Sensors/Fmu/Mpu9250/GyroZ_rads");
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


bool sim_imu_update() {
  const int sim_imu_size = 1024;
  char packet_buf[sim_imu_size];
  char *pt;

  bool fresh_data = false;

  int result;
  while ( (result = sock_imu.recv(packet_buf, sim_imu_size, 0)) > 0) {
    fresh_data = true;

    pt = strtok(packet_buf, ","); imuTime_s = atof(pt);
    pt = strtok(NULL, ","); imuTime_us = atof(pt);
    pt = strtok(NULL, ","); ax_mps2 = atof(pt);
    pt = strtok(NULL, ","); ay_mps2 = atof(pt);
    pt = strtok(NULL, ","); az_mps2 = atof(pt);
    pt = strtok(NULL, ","); p_rps = atof(pt);
    pt = strtok(NULL, ","); q_rps = atof(pt);
    pt = strtok(NULL, ","); r_rps = atof(pt);
    pt = strtok(NULL, ","); hx_uT = atof(pt);
    pt = strtok(NULL, ","); hy_uT = atof(pt);
    pt = strtok(NULL, ","); hz_uT = atof(pt);

    // std::cout << imuTime_s << "\t"
    //           << imuTime_us << "\t"
    //           << ax_mps2 << "\t"
    //           << ay_mps2 << "\t"
    //           << az_mps2 << "\t"
    //           << p_rps << "\t"
    //           << q_rps << "\t"
    //           << r_rps << "\t"
    //           << hx_uT << "\t"
    //           << hy_uT << "\t"
    //           << hz_uT << "\t"
    //           << std::endl;
  }

  imuTime_node->setDouble(imuTime_us);
  ax_node->setFloat(ax_mps2);
  ay_node->setFloat(ay_mps2);
  az_node->setFloat(az_mps2);
  p_node->setFloat(p_rps);
  q_node->setFloat(q_rps);
  r_node->setFloat(r_rps);
  hx_node->setFloat(hx_uT);
  hy_node->setFloat(hy_uT);
  hz_node->setFloat(hz_uT);

  return fresh_data;
}

void sim_imu_close() {
  sock_imu.close();
}

// GPS
static netSocket sock_gps;

static float gpsTime_s = 0.0;
static double gpsTime_us = 0.0;
static double lat_rad = 0.0;
static double lon_rad = 0.0;
static float alt_m = 0.0;
static float vn_mps = 0.0;
static float ve_mps = 0.0;
static float vd_mps = 0.0;
static uint8_t sats = 0;
static uint8_t fix = 0;

static ElementPtr gpsTime_node;
static ElementPtr lat_node;
static ElementPtr lon_node;
static ElementPtr alt_node;
static ElementPtr vn_node;
static ElementPtr ve_node;
static ElementPtr vd_node;
static ElementPtr sats_node;
static ElementPtr fix_node;

bool sim_gps_init() {
  printf("sim_gps_init()\n");
  // bind def tree pointers
  gpsTime_node = deftree.getElement("/Sensors/uBlox/Time_us");
  lon_node = deftree.getElement("/Sensors/uBlox/Longitude_rad");
  lat_node = deftree.getElement("/Sensors/uBlox/Latitude_rad");
  alt_node = deftree.getElement("/Sensors/uBlox/Altitude_m");
  vn_node = deftree.getElement("/Sensors/uBlox/NorthVelocity_ms");
  ve_node = deftree.getElement("/Sensors/uBlox/EastVelocity_ms");
  vd_node = deftree.getElement("/Sensors/uBlox/DownVelocity_ms");
  sats_node = deftree.getElement("/Sensors/uBlox/NumberSatellites");
  fix_node = deftree.getElement("/Sensors/uBlox/Fix");

  // open a UDP socket
  if ( ! sock_gps.open( false ) ) {
    printf("Error opening gps input socket\n");
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

bool sim_gps_update() {
  const int sim_gps_size = 1024;
  char packet_buf[sim_gps_size];
  char *pt;

  bool fresh_data = false;

  int result;
  while ( (result = sock_gps.recv(packet_buf, sim_gps_size, 0)) > 0 ) {
    fresh_data = true;

    pt = strtok(packet_buf, ","); gpsTime_s = atof(pt);
    pt = strtok(NULL, ","); gpsTime_us = atof(pt);
    pt = strtok(NULL, ","); lat_rad = atof(pt);
    pt = strtok(NULL, ","); lon_rad = atof(pt);
    pt = strtok(NULL, ","); alt_m = atof(pt);
    pt = strtok(NULL, ","); vn_mps = atof(pt);
    pt = strtok(NULL, ","); ve_mps = atof(pt);
    pt = strtok(NULL, ","); vd_mps = atof(pt);
    sats = 8;
    fix = 1;

    // std::cout << gpsTime_s << "\t"
    //           << gpsTime_us << "\t"
    //           << lat_rad << "\t"
    //           << lon_rad << "\t"
    //           << alt_m << "\t"
    //           << vn_mps << "\t"
    //           << ve_mps << "\t"
    //           << vd_mps << "\t"
    //           << std::endl;
  }

  // write values even if data isn't fresh (to overwrite real sensor data)
  gpsTime_node->setDouble(gpsTime_us);
  lon_node->setDouble(lon_rad);
  lat_node->setDouble(lat_rad);
  alt_node->setFloat(alt_m);
  vn_node->setFloat(vn_mps);
  ve_node->setFloat(ve_mps);
  vd_node->setFloat(vd_mps);
  sats_node->setInt(8);
  fix_node->setInt(fix);

  return fresh_data;
}

void sim_gps_close() {
  sock_gps.close();
}

// Pitot Static
static netSocket sock_pitot;

static float pitotTime_s = 0.0;
static double pitotTime_us = 0.0;
static float pitotPresStatic_pa = 0.0;
static float pitotTempStatic_C = 0.0;
static float pitotPresTip_pa = 0.0;
static float pitotTempTip_C = 0.0;

// static ElementPtr pitotTime_node;
static ElementPtr pitotPresStatic_node;
static ElementPtr pitotTempStatic_node;
static ElementPtr pitotPresTip_node;
static ElementPtr pitotTempTip_node;

bool sim_pitot_init() {
  printf("sim_pitot_init()\n");
  // pitotTime_node = deftree.getElement("/Sensors/Fmu/Time_us");
  pitotPresStatic_node = deftree.getElement("/Sensors/Pitot/Static/Pressure_Pa");
  pitotTempStatic_node = deftree.getElement("/Sensors/Pitot/Static/Temperature_C");
  pitotPresTip_node = deftree.getElement("/Sensors/Pitot/Differential/Pressure_Pa");
  pitotTempTip_node = deftree.getElement("/Sensors/Pitot/Differential/Temperature_C");

  // open a UDP socket
  if ( ! sock_pitot.open( false ) ) {
    printf("Error opening pitot input socket\n");
    return false;
  }

  // bind ...
  if ( sock_pitot.bind( "", port_pitot ) == -1 ) {
    printf("error binding to port %d\n", port_pitot );
    return false;
  }

  // don't block waiting for input
  sock_pitot.setBlocking( false );

  return true;
}

bool sim_pitot_update() {
  const int sim_pitot_size = 1024;
  char packet_buf[sim_pitot_size];
  char *pt;

  bool fresh_data = false;

  int result;
  while ( (result = sock_pitot.recv(packet_buf, sim_pitot_size, 0)) > 0 ) {
    fresh_data = true;

    pt = strtok(packet_buf, ","); pitotTime_s = atof(pt);
    // pt = strtok(NULL, ","); pitotTime_us = atof(pt);
    pt = strtok(NULL, ","); pitotPresStatic_pa = atof(pt);
    pt = strtok(NULL, ","); pitotTempStatic_C = atof(pt);
    pt = strtok(NULL, ","); pitotPresTip_pa = atof(pt);
    pt = strtok(NULL, ","); pitotTempTip_C = atof(pt);

    // std::cout << pitotTime_s << "\t"
    //           << pitotTime_us << "\t"
    //           << pitotPresStatic_pa << "\t"
    //           << pitotTempStatic_C << "\t"
    //           << pitotPresTip_pa << "\t"
    //           << pitotTempTip_C << "\t"
    //           << std::endl;
  }

  // write values even if data isn't fresh (to overwrite real sensor data)
  // pitotTime_node->setDouble(pitotTime_us);
  pitotPresStatic_node->setFloat(pitotPresStatic_pa);
  pitotTempStatic_node->setFloat(pitotTempStatic_C);
  pitotPresTip_node->setFloat(pitotPresTip_pa);
  pitotTempTip_node->setFloat(pitotTempTip_C);

  return fresh_data;
}

void sim_pitot_close() {
  sock_pitot.close();
}


// 5-Hole Probe - [Method1]
static netSocket sock_five1;

static float five1Time_s = 0.0;
static double five1Time_us = 0.0;
static float five1PresStatic_pa = 0.0;
static float five1TempStatic_C = 0.0;
static float five1PresTip_pa = 0.0;
static float five1TempTip_C = 0.0;
static float five1PresAlpha_pa = 0.0;
static float five1TempAlpha_C = 0.0;
static float five1PresBeta_pa = 0.0;
static float five1TempBeta_C = 0.0;

// static ElementPtr fiveholeTime_node;
static ElementPtr five1PresStatic_node;
static ElementPtr five1TempStatic_node;
static ElementPtr five1PresTip_node;
static ElementPtr five1TempTip_node;
static ElementPtr five1PresAlpha_node;
static ElementPtr five1TempAlpha_node;
static ElementPtr five1PresBeta_node;
static ElementPtr five1TempBeta_node;

bool sim_5hole1_init() {
  printf("sim_5hole1_init()\n");
  // five1Time_node = deftree.getElement("/Sensors/5Hole/Time_us");
  five1PresStatic_node = deftree.getElement("/Sensors/5Hole/Static/Pressure_Pa");
  five1TempStatic_node = deftree.getElement("/Sensors/5Hole/Static/Temperature_C");
  five1PresTip_node = deftree.getElement("/Sensors/5Hole/Tip/Pressure_Pa");
  five1TempTip_node = deftree.getElement("/Sensors/5Hole/Tip/Temperature_C");
  five1PresAlpha_node = deftree.getElement("/Sensors/5Hole/PresAlpha/Pressure_Pa");
  five1TempAlpha_node = deftree.getElement("/Sensors/5Hole/PresAlpha/Temperature_C");
  five1PresBeta_node = deftree.getElement("/Sensors/5Hole/PresBeta/Pressure_Pa");
  five1TempBeta_node = deftree.getElement("/Sensors/5Hole/PresBeta/Temperature_C");

  // open a UDP socket
  if ( ! sock_five1.open( false ) ) {
    printf("Error opening 5Hole1 input socket\n");
    return false;
  }

  // bind ...
  if ( sock_five1.bind( "", port_five1 ) == -1 ) {
    printf("error binding to port %d\n", port_five1 );
    return false;
  }

  // don't block waiting for input
  sock_five1.setBlocking( false );

  return true;
}

bool sim_5hole1_update() {
  const int sim_five1_size = 1024;
  char packet_buf[sim_five1_size];
  char *pt;

  bool fresh_data = false;

  int result;
  while ( (result = sock_five1.recv(packet_buf, sim_five1_size, 0)) > 0 ) {
    fresh_data = true;

    pt = strtok(packet_buf, ","); five1Time_s = atof(pt);
    // pt = strtok(NULL, ","); five1Time_us = atof(pt);
    pt = strtok(NULL, ","); five1PresStatic_pa = atof(pt);
    pt = strtok(NULL, ","); five1TempStatic_C = atof(pt);
    pt = strtok(NULL, ","); five1PresTip_pa = atof(pt);
    pt = strtok(NULL, ","); five1TempTip_C = atof(pt);
    pt = strtok(NULL, ","); five1PresAlpha_pa = atof(pt);
    pt = strtok(NULL, ","); five1TempAlpha_C = atof(pt);
    pt = strtok(NULL, ","); five1PresBeta_pa = atof(pt);
    pt = strtok(NULL, ","); five1TempBeta_C = atof(pt);

    // std::cout << five1Time_s << "\t"
    //           << five1Time_us << "\t"
    //           << five1PresStatic_pa << "\t"
    //           << five1TempStatic_C << "\t"
    //           << five1PresTip_pa << "\t"
    //           << five1TempTip_C << "\t"
    //           << five1PresAlpha_pa << "\t"
    //           << five1TempAlpha_C << "\t"
    //           << five1PresBeta_pa << "\t"
    //           << five1TempBeta_C << "\t"
    //           << std::endl;
  }

  // write values even if data isn't fresh (to overwrite real sensor data)
  // five1Time_node->setDouble(five1Time_us);
  five1PresStatic_node->setFloat(five1PresStatic_pa);
  five1TempStatic_node->setFloat(five1TempStatic_C);
  five1PresTip_node->setFloat(five1PresTip_pa);
  five1TempTip_node->setFloat(five1TempTip_C);
  five1PresAlpha_node->setFloat(five1PresAlpha_pa);
  five1TempAlpha_node->setFloat(five1TempAlpha_C);
  five1PresBeta_node->setFloat(five1PresBeta_pa);
  five1TempBeta_node->setFloat(five1TempBeta_C);

  return fresh_data;
}

void sim_5hole1_close() {
  sock_five1.close();
}


// 5-Hole Probe - [Method2]
static netSocket sock_five2;

static float five2Time_s = 0.0;
static double five2Time_us = 0.0;
static float five2PresStatic_pa = 0.0;
static float five2TempStatic_C = 0.0;
static float five2PresTip_pa = 0.0;
static float five2TempTip_C = 0.0;
static float five2PresAlpha_pa = 0.0;
static float five2TempAlpha_C = 0.0;
static float five2PresBeta_pa = 0.0;
static float five2TempBeta_C = 0.0;

// static ElementPtr fiveholeTime_node;
static ElementPtr five2PresStatic_node;
static ElementPtr five2TempStatic_node;
static ElementPtr five2PresTip_node;
static ElementPtr five2TempTip_node;
static ElementPtr five2PresAlpha_node;
static ElementPtr five2TempAlpha_node;
static ElementPtr five2PresBeta_node;
static ElementPtr five2TempBeta_node;

bool sim_5hole2_init() {
  printf("sim_5hole2_init()\n");
  // five2Time_node = deftree.getElement("/Sensors/5Hole/Time_us");
  five2PresStatic_node = deftree.getElement("/Sensors/5Hole/Static/Pressure_Pa");
  five2TempStatic_node = deftree.getElement("/Sensors/5Hole/Static/Temperature_C");
  five2PresTip_node = deftree.getElement("/Sensors/5Hole/Tip/Pressure_Pa");
  five2TempTip_node = deftree.getElement("/Sensors/5Hole/Tip/Temperature_C");
  five2PresAlpha_node = deftree.getElement("/Sensors/5Hole/PresAlpha/Pressure_Pa");
  five2TempAlpha_node = deftree.getElement("/Sensors/5Hole/PresAlpha/Temperature_C");
  five2PresBeta_node = deftree.getElement("/Sensors/5Hole/PresBeta/Pressure_Pa");
  five2TempBeta_node = deftree.getElement("/Sensors/5Hole/PresBeta/Temperature_C");

  // open a UDP socket
  if ( ! sock_five2.open( false ) ) {
    printf("Error opening 5Hole2 input socket\n");
    return false;
  }

  // bind ...
  if ( sock_five2.bind( "", port_five2 ) == -1 ) {
    printf("error binding to port %d\n", port_five2 );
    return false;
  }

  // don't block waiting for input
  sock_five2.setBlocking( false );

  return true;
}

bool sim_5hole2_update() {
  const int sim_five2_size = 1024;
  char packet_buf[sim_five2_size];
  char *pt;

  bool fresh_data = false;

  int result;
  while ( (result = sock_five2.recv(packet_buf, sim_five2_size, 0)) > 0 ) {
    fresh_data = true;

    pt = strtok(packet_buf, ","); five2Time_s = atof(pt);
    // pt = strtok(NULL, ","); five2Time_us = atof(pt);
    pt = strtok(NULL, ","); five2PresStatic_pa = atof(pt);
    pt = strtok(NULL, ","); five2TempStatic_C = atof(pt);
    pt = strtok(NULL, ","); five2PresTip_pa = atof(pt);
    pt = strtok(NULL, ","); five2TempTip_C = atof(pt);
    pt = strtok(NULL, ","); five2PresAlpha_pa = atof(pt);
    pt = strtok(NULL, ","); five2TempAlpha_C = atof(pt);
    pt = strtok(NULL, ","); five2PresBeta_pa = atof(pt);
    pt = strtok(NULL, ","); five2TempBeta_C = atof(pt);

    // std::cout << five2Time_s << "\t"
    //           << five2Time_us << "\t"
    //           << five2PresStatic_pa << "\t"
    //           << five2TempStatic_C << "\t"
    //           << five2PresTip_pa << "\t"
    //           << five2TempTip_C << "\t"
    //           << five2PresAlpha_pa << "\t"
    //           << five2TempAlpha_C << "\t"
    //           << five2PresBeta_pa << "\t"
    //           << five2TempBeta_C << "\t"
    //           << std::endl;
  }

  // write values even if data isn't fresh (to overwrite real sensor data)
  // five2Time_node->setDouble(five2Time_us);
  five2PresStatic_node->setFloat(five2PresStatic_pa);
  five2TempStatic_node->setFloat(five2TempStatic_C);
  five2PresTip_node->setFloat(five2PresTip_pa);
  five2TempTip_node->setFloat(five2TempTip_C);
  five2PresAlpha_node->setFloat(five2PresAlpha_pa);
  five2TempAlpha_node->setFloat(five2TempAlpha_C);
  five2PresBeta_node->setFloat(five2PresBeta_pa);
  five2TempBeta_node->setFloat(five2TempBeta_C);

  return fresh_data;
}

void sim_5hole2_close() {
  sock_five2.close();
}


// Actuator Commands
static netSocket sock_cmd;

static ElementPtr cmdTime_node;

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

bool sim_cmd_init() {
  printf("sim_cmd_init()\n");

  if (modelName == "mAEWing2") {
    cmdTime_node = deftree.getElement("/Sensors/Fmu/Time_us");
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
    cmdTime_node = deftree.getElement("/Sensors/Fmu/Time_us");
    cmdAilL_node = deftree.getElement("/Control/cmdAilL_rad");
    cmdAilR_node = deftree.getElement("/Control/cmdAilR_rad");
    cmdElev_node = deftree.getElement("/Control/cmdElev_rad");
    cmdRud_node = deftree.getElement("/Control/cmdRud_rad");
    cmdFlapL_node = deftree.getElement("/Control/cmdFlapL_rad");
    cmdFlapR_node = deftree.getElement("/Control/cmdFlapR_rad");
    cmdMotor_node = deftree.getElement("/Control/cmdMotor_nd");
  } else {
    std::cout << "JSBSim 'Model' not understood" << std::endl;
  }

  // open a UDP socket
  if ( ! sock_cmd.open( false ) ) {
    printf("Error opening act input socket\n");
    return false;
  }

  // connect ...
  if ( sock_cmd.connect( host_cmd.c_str(), port_cmd ) == -1 ) {
    printf("error connecting to %s:%d\n", host_cmd.c_str(), port_cmd);
    return false;
  }

  // don't block
  sock_cmd.setBlocking( false );

  return true;
}

bool sim_cmd_update() {
  std::string response = "";

  if (modelName == "mAEWing2") {
    float cmdTime_s = 1e-6 * cmdTime_node->getFloat();
    double cmdTime_us = cmdTime_node->getDouble();
    float cmdTE1L_rad = cmdTE1L_node->getFloat();
    float cmdTE1R_rad = cmdTE1R_node->getFloat();
    float cmdTE2L_rad = cmdTE2L_node->getFloat();
    float cmdTE2R_rad = cmdTE2R_node->getFloat();
    float cmdTE3L_rad = cmdTE3L_node->getFloat();
    float cmdTE3R_rad = cmdTE3R_node->getFloat();
    float cmdTE4L_rad = cmdTE4L_node->getFloat();
    float cmdTE4R_rad = cmdTE4R_node->getFloat();
    float cmdTE5L_rad = cmdTE5L_node->getFloat();
    float cmdTE5R_rad = cmdTE5R_node->getFloat();
    float cmdLEL_rad = cmdLEL_node->getFloat();
    float cmdLER_rad = cmdLER_node->getFloat();
    float cmdMotor_nd = cmdMotor_node->getFloat();
    float cmdGear_nd = cmdGear_node->getFloat();

    response += std::to_string(cmdTime_s) + ",";
    response += std::to_string(cmdTime_us) + ",";
    response += std::to_string(cmdTE1L_rad) + ",";
    response += std::to_string(cmdTE1R_rad) + ",";
    response += std::to_string(cmdTE2L_rad) + ",";
    response += std::to_string(cmdTE2R_rad) + ",";
    response += std::to_string(cmdTE3L_rad) + ",";
    response += std::to_string(cmdTE3R_rad) + ",";
    response += std::to_string(cmdTE4R_rad) + ",";
    response += std::to_string(cmdTE4R_rad) + ",";
    response += std::to_string(cmdTE5L_rad) + ",";
    response += std::to_string(cmdTE5R_rad) + ",";
    response += std::to_string(cmdLEL_rad) + ",";
    response += std::to_string(cmdLER_rad) + ",";
    response += std::to_string(cmdMotor_nd) + ",";
    response += std::to_string(cmdGear_nd) + "\r\n";

  } else if ((modelName == "UltraStick25e") || (modelName == "UltraStick120")) {

    float cmdTime_s = 1e-6 * cmdTime_node->getFloat();
    double cmdTime_us = cmdTime_node->getDouble();
    float cmdAilL_rad = cmdAilL_node->getFloat();
    float cmdAilR_rad = cmdAilR_node->getFloat();
    float cmdElev_rad = cmdElev_node->getFloat();
    float cmdRud_rad = cmdRud_node->getFloat();
    float cmdFlapL_rad = cmdFlapL_node->getFloat();
    float cmdFlapR_rad = cmdFlapR_node->getFloat();
    float cmdMotor_nd = cmdMotor_node->getFloat();

    response += std::to_string(cmdTime_s) + ",";
    response += std::to_string(cmdTime_us) + ",";
    response += std::to_string(cmdAilL_rad) + ",";
    response += std::to_string(cmdAilR_rad) + ",";
    response += std::to_string(cmdElev_rad) + ",";
    response += std::to_string(cmdRud_rad) + ",";
    response += std::to_string(cmdFlapL_rad) + ",";
    response += std::to_string(cmdFlapR_rad) + ",";
    response += std::to_string(cmdMotor_nd) + "\r\n";

    // std::cout << cmdTime_s << "\t"
    //           << cmdTime_us << "\t"
    //           << cmdMotor_nd << "\t"
    //           << std::endl;

  } else {
    std::cout << "JSBSim 'Model' not understood" << std::endl;
  }

  int result = sock_cmd.send( response.c_str(), response.length(), 0 );
  if ( result != response.length() ) {
    return false;
  }

  return true;
}

void sim_cmd_close() {
  sock_cmd.close();
}
