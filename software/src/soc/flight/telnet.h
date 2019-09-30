/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor, Chris Regan, Curt Olson (original author)
*/

#pragma once

#include "netChannel.h"

/**
 * Telent server class.
 * This class provides a telnet-like server for remote access to
 * FlightGear properties.
 */
class UGTelnet: netChannel
{

private:

  /**
   * Server port to listen on.
   */
  int port;
  bool enabled;

public:

  /**
   * Create a new TCP server.
   *
   * @param tokens Tokenized configuration parameters
   */
  UGTelnet( const int port_num );

  /**
   * Destructor.
   */
  ~UGTelnet();

  /**
   * Start the telnet server.
   */
  bool open();

  /**
   * Process network activity.
   */
  bool process();

  /**
   *
   */
  bool close();

  /**
   * Accept a new client connection.
   */
  void handleAccept();

};
