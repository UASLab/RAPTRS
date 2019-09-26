/*
Copyright (c) 2018 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "telemetry.h"
#include <iostream>
#include <iomanip>
#include <stdint.h>

int main(int argc, char* argv[]) {
  std::cout << "Bolder Flight Systems" << std::endl;
  std::cout << "Telemetry Server Version 1.0.0 " << std::endl << std::endl;

  /* declare classes */
  TelemetryServer Telemetry;

  while(1) {
    Telemetry.ReceivePacket();
  }

	return 0;
}
