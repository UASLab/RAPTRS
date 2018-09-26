
#ifndef TELEMETRY_HXX_
#define TELEMETRY_HXX_

#include "definition-tree.hxx"
#include "hardware-defs.hxx"
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

#pragma pack(push, 1)
struct pilotPacket
{
   uint8_t index;
   double time;
   int16_t chan[8];
   int8_t status;
};

struct airPacket //BdHhhffhHBBB
{
   uint8_t index;
   double timestamp;
   uint16_t pressure_mbar;
   int16_t temp_degC;
   int16_t airspeed_smoothed_kt;
   float altitude_smoothed_m;
   float altitude_true_m;
   int16_t pressure_vertical_speed_fps;
   uint16_t wind_dir_deg;
   uint8_t wind_speed_kt;
   uint8_t pitot_scale_factor;
   uint8_t status;
};

struct filterPacket //BdddfhhhhhhhhhhhhBB
{
   uint8_t index;
   double timestamp;
   double latitude_deg;
   double longitude_deg;
   float altitude_m;
   int16_t vn_ms;
   int16_t ve_ms;
   int16_t vd_ms;
   int16_t roll_deg;
   int16_t pitch_deg;
   int16_t heading_deg;
   int16_t p_bias;
   int16_t q_bias;
   int16_t r_bias;
   int16_t ax_bias;
   int16_t ay_bias;
   int16_t az_bias;
   uint8_t sequence_num;
   uint8_t status; //always 0
};

struct ImunodePacket //BdfffffffffhB
{
   uint8_t index;
   double imu_timestamp;
   float p_rad_sec;
   float q_rad_sec;
   float r_rad_sec;
   float ax_mps_sec;
   float ay_mps_sec;
   float az_mps_sec;
   float hx;
   float hy;
   float hz;
   int16_t temp_C;
   uint8_t status;
};

struct numactPacket //BdhhHhhhhhB
{
   uint8_t index; //always 0
   double timestamp;
   int16_t aileron;
   int16_t elevator;
   uint16_t throttle;
   int16_t rudder;
   int16_t channel5;
   int16_t flaps;
   int16_t channel7;
   int16_t channel8;
   uint8_t status; //always 0
};

struct healthPacket //BfHHHHHH
{
   uint8_t index;
   float frame_time;
   uint16_t system_load_avg;
   uint16_t board_vcc;
   uint16_t extern_volts;
   uint16_t extern_cell_volts;
   uint16_t extern_amps;
   uint16_t dekamah;
};

struct payloadPacket //BdH
{
   uint8_t index;
   double timestamp;
   uint16_t trigger_num;
};

struct gpsPacket
{
   uint8_t index;
   double timestamp, lat_deg, long_deg;
   float alt_m;
   int16_t vn_ms;
   int16_t ve_ms;
   int16_t vd_ms;
   double unix_time_sec;
   uint8_t satellites;
   uint16_t horiz_accuracy_m;
   uint16_t vert_accuracy_m;
   uint16_t pdop;
   uint8_t fixType;
};

struct ap_status
{
   uint8_t index;
   double frame_time;
   uint8_t flags; //?
   int16_t groundtrack_deg;
   int16_t roll_deg;
   uint16_t Target_msl_ft;
   int16_t ground_m;
   int16_t pitch_deg;
   int16_t airspeed_kt;
   uint16_t flight_timer;
   uint16_t target_waypoint_idx;
   double wp_lon;
   double wp_lat;
   uint16_t wp_index;
   uint16_t routesize;
   uint8_t sequence_num;
};
#pragma pack(pop)

class TelemetryClient {
  public:
    TelemetryClient();
    void Configure(const rapidjson::Value& Config,DefinitionTree *DefinitionTreePtr);
    void Send();
    void End();
  private:
    std::string _RootPath = "/Telemetry";
    std::string Uart;
    uint64_t Baud;
    enum PacketType_ {
      UartPacket,
      BaudPacket,
      DataPacket
    };
    int TelemetrySocket_;
    int TelemetryPort_ = 8020;
    struct sockaddr_in TelemetryServer_;
  bool useTime, useStaticPressure, useAirspeed, useAlt, useGps, useSbus, useImu, useAttitude, usePower;
    struct TimeDataPtr{
      uint64_t* Time_us;
    };
    struct StaticPressDataPtr{
      float *Pressure_Pa;
      float *Temperature_C;
    };
    struct AirspeedDataPtr{
      float *Airspeed_ms;
    };
    struct AltDataPtr{
      float *Alt_m;
    };
    struct GpsDataPtr{
      bool *Fix;                                 // True for 3D fix only
      uint8_t *NumberSatellites;                 // Number of satellites used in solution
      uint32_t *TOW;                             // GPS time of the navigation epoch
      uint16_t *Year;                            // UTC year
      uint8_t *Month;                            // UTC month
      uint8_t *Day;                              // UTC day
      uint8_t *Hour;                             // UTC hour
      uint8_t *Min;                              // UTC minute
      uint8_t *Sec;                              // UTC second
      double *Lat;
      double *Lon;
      double *Alt;
      double *Vn;
      double *Ve;
      double *Vd;
      double *HAcc;
      double *VAcc;
      double *SAcc;
      double *pDOP;                              // Position DOP
    };
    struct SbusDataPtr{
      float *Channels[16];
      bool *FailSafe;
      uint64_t *LostFrames;
    };
    struct ImuDataPtr{
      float *Ax, *Ay, *Az;
      float *Gx, *Gy, *Gz;
      float *Hx, *Hy, *Hz;
      float *Temperature_C;                      // Temperature, C
    };
    struct AttitudeDataPtr{
      float *Ax,*Ay,*Az;
      float *Gx,*Gy,*Gz;
      float *Axb,*Ayb,*Azb;
      float *Gxb,*Gyb,*Gzb;
      float *Pitch,*Roll,*Yaw,*Heading,*Track;
      double *Lat,*Lon,*Alt;
      float *Vn,*Ve,*Vd;
    };
  struct PowerDataPtr{
    float *MinCellVolt;
  };
    struct DataPtr{
      TimeDataPtr Time;
      StaticPressDataPtr StaticPress;
      AirspeedDataPtr Airspeed;
      AltDataPtr Alt;
      GpsDataPtr Gps;
      SbusDataPtr Sbus;
      ImuDataPtr Imu;
      AttitudeDataPtr Attitude;
      PowerDataPtr Power;
    };
    struct TimeData{
      uint64_t Time_us;
    };
    struct StaticPressData{
      float Pressure_Pa;
      float Temperature_C;
    };
    struct AirspeedData{
      float Airspeed_ms;
    };
    struct AltData{
      float Alt_m;
    };
    struct GpsData{
      bool Fix;                                 // True for 3D fix only
      uint8_t NumberSatellites;                 // Number of satellites used in solution
      uint32_t TOW;                             // GPS time of the navigation epoch
      uint16_t Year;                            // UTC year
      uint8_t Month;                            // UTC month
      uint8_t Day;                              // UTC day
      uint8_t Hour;                             // UTC hour
      uint8_t Min;                              // UTC minute
      uint8_t Sec;                              // UTC second
      double Lat;
      double Lon;
      double Alt;
      double Vn;
      double Ve;
      double Vd;
      double HAcc;
      double VAcc;
      double SAcc;
      double pDOP;                              // Position DOP
    };
    struct SbusData{
      float Channels[16];
      bool FailSafe;
      uint64_t LostFrames;
    };
    struct ImuData{
      float Ax, Ay, Az;
      float Gx, Gy, Gz;
      float Hx, Hy, Hz;
      float Temperature_C;                      // Temperature, C
    };
    struct AttitudeData{
      float Ax,Ay,Az;
      float Gx,Gy,Gz;
      float Axb,Ayb,Azb;
      float Gxb,Gyb,Gzb;
      float Pitch,Roll,Yaw,Heading,Track;
      double Lat,Lon,Alt;
      double Vn,Ve,Vd;
    };
  struct PowerData{
    float MinCellVolt;
    int ExperimentNumber;
    float ExperimentProgress;
  };
    struct Data{
      TimeData Time;
      StaticPressData StaticPress;
      AirspeedData Airspeed;
      AltData Alt;
      GpsData Gps;
      SbusData Sbus;
      ImuData Imu;
      AttitudeData Attitude;
      PowerData Power;
    };
    DataPtr DataPtr_;
    Data Data_;
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];
    uint16_t ParserState_ = 0;
    void SendPacket(PacketType_ Type, std::vector<uint8_t> &Buffer);
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
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
      UartPacket,
      BaudPacket,
      DataPacket
    };
    struct TimeData{
      uint64_t Time_us;
    };
    struct StaticPressData{
      float Pressure_Pa;
      float Temperature_C;
    };
    struct AirspeedData{
      float Airspeed_ms;
    };
    struct AltData{
      float Alt_m;
    };
    struct GpsData{
      bool Fix;                                 // True for 3D fix only
      uint8_t NumberSatellites;                 // Number of satellites used in solution
      uint32_t TOW;                             // GPS time of the navigation epoch
      uint16_t Year;                            // UTC year
      uint8_t Month;                            // UTC month
      uint8_t Day;                              // UTC day
      uint8_t Hour;                             // UTC hour
      uint8_t Min;                              // UTC minute
      uint8_t Sec;                              // UTC second
      double Lat;
      double Lon;
      double Alt;
      double Vn;
      double Ve;
      double Vd;
      double HAcc;
      double VAcc;
      double SAcc;
      double pDOP;                              // Position DOP
    };
    struct SbusData{
      float Channels[16];
      bool FailSafe;
      uint64_t LostFrames;
    };
    struct ImuData{
      float Ax, Ay, Az;
      float Gx, Gy, Gz;
      float Hx, Hy, Hz;
      float Temperature_C;                      // Temperature, C
    };
    struct AttitudeData{
      float Ax,Ay,Az;
      float Gx,Gy,Gz;
      float Axb,Ayb,Azb;
      float Gxb,Gyb,Gzb;
      float Pitch,Roll,Yaw,Heading,Track;
      double Lat,Lon,Alt;
      double Vn,Ve,Vd;
    };
  struct PowerData{
    float MinCellVolt;
  };
    struct Data{
      TimeData Time;
      StaticPressData StaticPress;
      AirspeedData Airspeed;
      AltData Alt;
      GpsData Gps;
      SbusData Sbus;
      ImuData Imu;
      AttitudeData Attitude;
      PowerData Power;
    };
    Data Data_;
    std::string Uart;
    uint64_t Baud;
    int TelemetrySocket_;
    int TelemetryPort_ = 8020;
    struct sockaddr_in TelemetryServer_;
    std::vector<uint8_t> Buffer;
    uint8_t Buffer_[kUartBufferMaxSize];
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t RxByte_;
    uint16_t ParserState_ = 0;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];

   int count;
   int num2 = 1;

   void update(const Data &DataRef);

   void generate_cksum(uint8_t id, uint8_t size, uint8_t * buf, uint8_t & cksum0, uint8_t &cksum1);
   void send_packet(uint8_t * package, uint8_t IDnum, uint8_t size);

    bool ParseMessage(uint8_t byte,PacketType_ *message,std::vector<uint8_t> *Payload);
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};

#endif
