
#ifndef TELEMETRY_HXX_
#define TELEMETRY_HXX_

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
    void End();
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
    struct TimeNodes{
      ElementPtr Time_us;
    };
    struct StaticPressNodes{
      ElementPtr Pressure_Pa;
      ElementPtr Temperature_C;
    };
    struct AirspeedNodes{
      ElementPtr Airspeed_ms;
    };
    struct AltNodes{
      ElementPtr Alt_m;
    };
    struct GpsNodes{
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
    };
    struct SbusNodes{
      ElementPtr Channels[16];
      ElementPtr FailSafe;
      ElementPtr LostFrames;
    };
    struct ImuNodes{
      ElementPtr Ax, Ay, Az;
      ElementPtr Gx, Gy, Gz;
      ElementPtr Hx, Hy, Hz;
      ElementPtr Temperature_C;                      // Temperature, C
    };
    struct AttitudeNodes{
      ElementPtr Ax, Ay, Az;
      ElementPtr Gx, Gy, Gz;
      ElementPtr Axb, Ayb, Azb;
      ElementPtr Gxb, Gyb, Gzb;
      ElementPtr Pitch, Roll, Yaw, Heading, Track;
      ElementPtr Lat, Lon, Alt;
      ElementPtr Vn, Ve, Vd;
    };
  struct PowerNodes{
    ElementPtr MinCellVolt;
  };
    struct DataNodes{
      TimeNodes Time;
      StaticPressNodes StaticPress;
      AirspeedNodes Airspeed;
      AltNodes Alt;
      GpsNodes Gps;
      SbusNodes Sbus;
      ImuNodes Imu;
      AttitudeNodes Attitude;
      PowerNodes Power;
    };
    DataNodes Nodes_;
    int count = 0;
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];
    void SendPacket(uint16_t pkt_id, uint8_t *Buffer, uint16_t len);
};

class TelemetryServer {
  public:
    TelemetryServer();
    void ReceivePacket();
    void End();
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
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t RxByte_;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];

    void generate_cksum(uint8_t id, uint8_t size, uint8_t * buf, uint8_t & cksum0, uint8_t &cksum1);
    void send_packet(uint8_t id, uint8_t *payload, uint8_t len);
};

#endif
