//
// FILE: sim-interface.hxx
// DESCRIPTION: aquire live sensor data from an running copy of JSBSim
//

#pragma once

#include "rapidjson/document.h"

// init everything
bool sim_init( const rapidjson::Value& Config );
bool sim_sensor_update();

// function prototypes
bool sim_imu_init();
bool sim_imu_update();
void sim_imu_close();

bool sim_gps_init();
bool sim_gps_update();
void sim_gps_close();

bool sim_pitot_init();
bool sim_pitot_update();
void sim_pitot_close();

bool sim_5hole1_init();
bool sim_5hole1_update();
void sim_5hole1_close();

bool sim_5hole2_init();
bool sim_5hole2_update();
void sim_5hole2_close();

bool sim_cmd_init();
bool sim_cmd_update();
void sim_cmd_close();
