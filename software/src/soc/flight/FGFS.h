//
// FILE: FGFS.hxx
// DESCRIPTION: aquire live sensor data from an running copy of Flightgear
//

#pragma once

#include "definition-tree.h"

// function prototypes
bool fgfs_imu_init( DefinitionTree *DefTree );
bool fgfs_imu_update();
void fgfs_imu_close();

bool fgfs_airdata_init();
bool fgfs_airdata_update();
void fgfs_airdata_close();

bool fgfs_gps_init( DefinitionTree *DefTree );
bool fgfs_gps_update();
void fgfs_gps_close();

bool fgfs_act_init( DefinitionTree *DefTree );
bool fgfs_act_update();
void fgfs_act_close();
