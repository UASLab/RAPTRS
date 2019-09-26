/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Curt Olson
*/

#pragma once

#include "definition-tree2.h"
#include "hardware-defs.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <exception>
#include <stdexcept>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <Eigen/Dense>

class TelemetryClient {
  public:
    TelemetryClient();
    void Configure(const rapidjson::Value& Config);
    void Send();
    // void End();
  private:
    std::string _RootPath = "/Telemetry";
    std::string Uart;
    uint32_t Baud;
    enum PacketType_ {
      UartPacket = 0,
      BaudPacket = 1
    };
    int TelemetrySocket_;
    int TelemetryPort_ = 8020;
    struct sockaddr_in TelemetryServer_;
  bool useTime, useStaticPressure, useAirspeed, useAlt, useGps, useSbus, useImu, useAttitude, usePower;
    struct {
      ElementPtr Time_us;
    } TimeNodes;
    struct {
      ElementPtr Pressure_Pa;
      ElementPtr Temperature_C;
    } StaticPressNodes;
    struct {
      ElementPtr Airspeed_ms;
    } AirspeedNodes;
    struct {
      ElementPtr Alt_m;
    } AltNodes;
    struct {
      ElementPtr Fix;                                 // True for 3D fix only
      ElementPtr NumberSatellites;                 // Number of satellites used in solution
      ElementPtr TOW;                             // GPS time of the navigation epoch
      ElementPtr Year;                            // UTC year
      ElementPtr Month;                            // UTC month
      ElementPtr Day;                              // UTC day
      ElementPtr Hour;                             // UTC hour
      ElementPtr Min;                              // UTC minute
      ElementPtr Sec;                              // UTC second
      ElementPtr Lat;
      ElementPtr Lon;
      ElementPtr Alt;
      ElementPtr Vn;
      ElementPtr Ve;
      ElementPtr Vd;
      ElementPtr HAcc;
      ElementPtr VAcc;
      ElementPtr SAcc;
      ElementPtr pDOP;                              // Position DOP
    } GpsNodes;
    struct {
      ElementPtr Channels[16];
      ElementPtr FailSafe;
      ElementPtr LostFrames;
    } SbusNodes;
    struct {
      ElementPtr Ax, Ay, Az;
      ElementPtr Gx, Gy, Gz;
      ElementPtr Hx, Hy, Hz;
      ElementPtr Temperature_C;                      // Temperature, C
    } ImuNodes;
    struct {
      ElementPtr Ax, Ay, Az;
      ElementPtr Gx, Gy, Gz;
      ElementPtr Axb, Ayb, Azb;
      ElementPtr Gxb, Gyb, Gzb;
      ElementPtr Pitch, Roll, Yaw, Heading, Track;
      ElementPtr Lat, Lon, Alt;
      ElementPtr Vn, Ve, Vd;
    } AttitudeNodes;
    struct {
      ElementPtr InputVolt;
      ElementPtr AvionicsVolt;
      ElementPtr MinCellVolt;
    } PowerNodes;
    int count = 0;
    void SendPacket(uint16_t pkt_id, uint8_t *Buffer, uint16_t len);
};

class TelemetryServer {
  public:
    TelemetryServer();
    void ReceivePacket();
    // void End();
  private:
    int FileDesc_;
    bool rxUart = false;
    bool rxBaud = false;
    bool uartLatch = false;
    enum PacketType_ {
      UartPacket = 0,
      BaudPacket = 1,
    };
    std::string Uart;
    uint32_t Baud;
    int TelemetrySocket_;
    int TelemetryPort_ = 8020;
    struct sockaddr_in TelemetryServer_;
    std::vector<uint8_t> Buffer;
    uint8_t Buffer_[kUartBufferMaxSize];
    uint8_t RxByte_;

    // external serial port link
    int state = 0;
    int counter = 0;
    uint8_t cksum_lo = 0, cksum_hi = 0;
    static const uint8_t MAX_MESSAGE_LEN = 200;
    static const uint8_t START_OF_MSG0 = 147;
    static const uint8_t START_OF_MSG1 = 224;
    uint32_t parse_errors = 0;
    uint8_t pkt_id = 0;
    uint8_t pkt_len = 0;
    uint8_t payload[MAX_MESSAGE_LEN];
    uint8_t sequence_num;
    void generate_cksum(uint8_t id, uint8_t size, uint8_t *buf, uint8_t & cksum0, uint8_t &cksum1);
    void send_message(uint8_t id, uint8_t *payload, uint8_t len);
    bool read_message();
    bool process_message();
    vector<string> split( const string& str, const char* sep, int maxsplit=0 );
};
