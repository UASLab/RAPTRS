/*
Copyright (c) 2018 - 2020 Regents of the University of Minnesota.
MIT License; See LICENSE.md for complete details
Author: Chris Regan, Curt Olson
*/

#include "simMgr.h"

// Sim Configure
bool SimMgr::Configure( const rapidjson::Value& Config ) {
  if ( Config.HasMember("JSBSim") ) {
    const rapidjson::Value& JsbConfig = Config["JSBSim"];
    
    LoadVal(JsbConfig, "SimFmuPort", &FmuPort, true);
    LoadVal(JsbConfig, "SimFmuBaud", &FmuBaud, true);

    std::cout << "Sim FMU interface on Port: " << FmuPort  << " Baud: " << FmuBaud << std::endl;
    std::cout << std::endl;

    return true;
  } else {
    return false;
  }
}
