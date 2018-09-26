#include <iostream>
using std::cout;
using std::endl;

#include "telemetry.hxx"

/* Opens a socket for telemetry */
TelemetryClient::TelemetryClient() {
  TelemetrySocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  TelemetryServer_.sin_family = AF_INET;
  TelemetryServer_.sin_port = htons(TelemetryPort_);
  TelemetryServer_.sin_addr.s_addr = inet_addr("127.0.0.1");
}

void TelemetryClient::Configure(const rapidjson::Value& Config,DefinitionTree *DefinitionTreePtr) {
  std::vector<uint8_t> Buffer;
  if (Config.HasMember("Uart")) {
    Uart = Config["Uart"].GetString();
    Buffer.resize(Uart.size());
    memcpy(Buffer.data(),Uart.data(),Buffer.size());
    SendPacket(UartPacket,Buffer);
  } else {
    throw std::runtime_error(std::string("ERROR")+_RootPath+std::string(": Uart not specified in configuration."));
  }
  if (Config.HasMember("Baud")) {
    Baud = Config["Baud"].GetUint64();
    Buffer.resize(sizeof(Baud));
    memcpy(Buffer.data(),&Baud,Buffer.size());
    SendPacket(BaudPacket,Buffer);
  } else {
    throw std::runtime_error(std::string("ERROR")+_RootPath+std::string(": Baud not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    DataPtr_.Time.Time_us = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
    useTime = true;
  }
  if (Config.HasMember("Static-Pressure")) {
    std::string Sensor = Config["Static-Pressure"].GetString();
    DataPtr_.StaticPress.Pressure_Pa = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Pressure_Pa");
    DataPtr_.StaticPress.Temperature_C = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Temperature_C");
    useStaticPressure = true;
  }
  if (Config.HasMember("Airspeed")) {
    DataPtr_.Airspeed.Airspeed_ms = DefinitionTreePtr->GetValuePtr<float*>(Config["Airspeed"].GetString());
    useAirspeed = true;
  }
  if (Config.HasMember("Altitude")) {
    DataPtr_.Alt.Alt_m = DefinitionTreePtr->GetValuePtr<float*>(Config["Altitude"].GetString());
    useAlt = true;
  }
  if (Config.HasMember("Filter")) {
    std::string Sensor = Config["Filter"].GetString();
    DataPtr_.Attitude.Ax = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelX_mss");
    DataPtr_.Attitude.Axb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelXBias_mss");
    DataPtr_.Attitude.Ay = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelY_mss");
    DataPtr_.Attitude.Ayb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelYBias_mss");
    DataPtr_.Attitude.Az = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelZ_mss");
    DataPtr_.Attitude.Azb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelZBias_mss");
    DataPtr_.Attitude.Gx = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroX_rads");
    DataPtr_.Attitude.Gxb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroXBias_rads");
    DataPtr_.Attitude.Gy = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroY_rads");
    DataPtr_.Attitude.Gyb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroYBias_rads");
    DataPtr_.Attitude.Gz = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroZ_rads");
    DataPtr_.Attitude.Gzb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroZBias_rads");
    DataPtr_.Attitude.Pitch = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Pitch_rad");
    DataPtr_.Attitude.Roll = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Roll_rad");
    DataPtr_.Attitude.Yaw = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Yaw_rad");
    DataPtr_.Attitude.Heading = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Heading_rad");
    DataPtr_.Attitude.Track = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Track_rad");
    DataPtr_.Attitude.Lon = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Longitude_rad");
    DataPtr_.Attitude.Lat = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Latitude_rad");
    DataPtr_.Attitude.Alt = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Altitude_m");
    DataPtr_.Attitude.Vn= DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/NorthVelocity_ms");
    DataPtr_.Attitude.Ve = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/EastVelocity_ms");
    DataPtr_.Attitude.Vd = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/DownVelocity_ms");
    useAttitude = true;
  }
  if (Config.HasMember("Gps")) {
    std::string Sensor = Config["Gps"].GetString();
    DataPtr_.Gps.Fix = (bool*)DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Fix");
    DataPtr_.Gps.NumberSatellites = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/NumberSatellites");
    DataPtr_.Gps.TOW = DefinitionTreePtr->GetValuePtr<uint32_t*>(Sensor+"/TOW");
    DataPtr_.Gps.Year = DefinitionTreePtr->GetValuePtr<uint16_t*>(Sensor+"/Year");
    DataPtr_.Gps.Month = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Month");
    DataPtr_.Gps.Day = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Day");
    DataPtr_.Gps.Hour = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Hour");
    DataPtr_.Gps.Min = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Minute");
    DataPtr_.Gps.Sec = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Second");
    DataPtr_.Gps.Lat = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Latitude_rad");
    DataPtr_.Gps.Lon = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Longitude_rad");
    DataPtr_.Gps.Alt = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Altitude_m");
    DataPtr_.Gps.Vn = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/NorthVelocity_ms");
    DataPtr_.Gps.Ve = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/EastVelocity_ms");
    DataPtr_.Gps.Vd = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/DownVelocity_ms");
    DataPtr_.Gps.HAcc = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/HorizontalAccuracy_m");
    DataPtr_.Gps.VAcc = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/VerticalAccuracy_m");
    DataPtr_.Gps.SAcc = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/VelocityAccuracy_ms");
    DataPtr_.Gps.pDOP = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/pDOP");
    useGps = true;
  }
  if (Config.HasMember("Imu")) {
    std::string Sensor = Config["Imu"].GetString();
    DataPtr_.Imu.Ax = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelX_mss");
    DataPtr_.Imu.Ay = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelY_mss");
    DataPtr_.Imu.Az = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelZ_mss");
    DataPtr_.Imu.Gx = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroX_rads");
    DataPtr_.Imu.Gy = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroY_rads");
    DataPtr_.Imu.Gz = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroZ_rads");
    DataPtr_.Imu.Temperature_C = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Temperature_C");
    useImu = true;
  }
  if (Config.HasMember("Sbus")) {
    std::string Sensor = Config["Sbus"].GetString();
    DataPtr_.Sbus.FailSafe = (bool*) DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/FailSafe");
    DataPtr_.Sbus.LostFrames = DefinitionTreePtr->GetValuePtr<uint64_t*>(Sensor+"/LostFrames");
    for (size_t j=0; j < 16; j++) {
      DataPtr_.Sbus.Channels[j] = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Channels/"+std::to_string(j));
    }
    useSbus = true;
  }
  if (Config.HasMember("Power")) {
    std::string Power = Config["Power"].GetString();
    DataPtr_.Power.MinCellVolt = (float*) DefinitionTreePtr->GetValuePtr<float*>(Power+"/MinCellVolt_V");
    usePower = true;
  }
}

void TelemetryClient::Send() {
  std::vector<uint8_t> DataPayload;
  if (useTime) {
    Data_.Time.Time_us = *DataPtr_.Time.Time_us;
  }
  if (useStaticPressure) {
    Data_.StaticPress.Pressure_Pa = *DataPtr_.StaticPress.Pressure_Pa;
    Data_.StaticPress.Temperature_C = *DataPtr_.StaticPress.Temperature_C;
  }
  if (useAirspeed) {
    Data_.Airspeed.Airspeed_ms = *DataPtr_.Airspeed.Airspeed_ms;
  }
  if (useAlt) {
    Data_.Alt.Alt_m = *DataPtr_.Alt.Alt_m;
  }
  if (useAttitude) {
    Data_.Attitude.Ax = *(DataPtr_.Attitude.Ax);
    Data_.Attitude.Axb = *DataPtr_.Attitude.Axb;
    Data_.Attitude.Ay = *DataPtr_.Attitude.Ay;
    Data_.Attitude.Ayb = *DataPtr_.Attitude.Ayb;
    Data_.Attitude.Az = *DataPtr_.Attitude.Az;
    Data_.Attitude.Azb = *DataPtr_.Attitude.Azb;
    Data_.Attitude.Gx = *DataPtr_.Attitude.Gx;
    Data_.Attitude.Gxb = *DataPtr_.Attitude.Gxb;
    Data_.Attitude.Gy = *DataPtr_.Attitude.Gy;
    Data_.Attitude.Gyb = *DataPtr_.Attitude.Gyb;
    Data_.Attitude.Gz = *DataPtr_.Attitude.Gz;
    Data_.Attitude.Gzb = *DataPtr_.Attitude.Gzb;
    Data_.Attitude.Pitch = *DataPtr_.Attitude.Pitch;
    Data_.Attitude.Roll = *DataPtr_.Attitude.Roll;
    Data_.Attitude.Yaw = *DataPtr_.Attitude.Yaw;
    Data_.Attitude.Heading = *DataPtr_.Attitude.Heading;
    Data_.Attitude.Track = *DataPtr_.Attitude.Track;
    Data_.Attitude.Lon = *DataPtr_.Attitude.Lon;
    Data_.Attitude.Lat = *DataPtr_.Attitude.Lat;
    Data_.Attitude.Alt = *DataPtr_.Attitude.Alt;
    Data_.Attitude.Vn= *DataPtr_.Attitude.Vn;
    Data_.Attitude.Ve = *DataPtr_.Attitude.Ve;
    Data_.Attitude.Vd = *DataPtr_.Attitude.Vd;
  }
  if (useGps) {
    Data_.Gps.Fix = *DataPtr_.Gps.Fix;
    Data_.Gps.NumberSatellites = *DataPtr_.Gps.NumberSatellites;
    Data_.Gps.TOW = *DataPtr_.Gps.TOW;
    Data_.Gps.Year = *DataPtr_.Gps.Year;
    Data_.Gps.Month = *DataPtr_.Gps.Month;
    Data_.Gps.Day = *DataPtr_.Gps.Day;
    Data_.Gps.Hour = *DataPtr_.Gps.Hour;
    Data_.Gps.Min = *DataPtr_.Gps.Min;
    Data_.Gps.Sec = *DataPtr_.Gps.Sec;
    Data_.Gps.Lat = *DataPtr_.Gps.Lat;
    Data_.Gps.Lon = *DataPtr_.Gps.Lon;
    Data_.Gps.Alt = *DataPtr_.Gps.Alt;
    Data_.Gps.Vn = *DataPtr_.Gps.Vn;
    Data_.Gps.Ve = *DataPtr_.Gps.Ve;
    Data_.Gps.Vd = *DataPtr_.Gps.Vd;
    Data_.Gps.HAcc = *DataPtr_.Gps.HAcc;
    Data_.Gps.VAcc = *DataPtr_.Gps.VAcc;
    Data_.Gps.SAcc = *DataPtr_.Gps.SAcc;
    Data_.Gps.pDOP = *DataPtr_.Gps.pDOP;
  }
  if (useImu) {
    Data_.Imu.Ax = *DataPtr_.Imu.Ax;
    Data_.Imu.Ay = *DataPtr_.Imu.Ay;
    Data_.Imu.Az = *DataPtr_.Imu.Az;
    Data_.Imu.Gx = *DataPtr_.Imu.Gx;
    Data_.Imu.Gy = *DataPtr_.Imu.Gy;
    Data_.Imu.Gz = *DataPtr_.Imu.Gz;
    Data_.Imu.Temperature_C = *DataPtr_.Imu.Temperature_C;
  }
  if (useSbus) {
    Data_.Sbus.FailSafe = *DataPtr_.Sbus.FailSafe;
    Data_.Sbus.LostFrames = *DataPtr_.Sbus.LostFrames;
    for (size_t j=0; j < 16; j++) {
      Data_.Sbus.Channels[j] = *DataPtr_.Sbus.Channels[j];
    }
  }
  if (usePower) {
    Data_.Power.MinCellVolt = *DataPtr_.Power.MinCellVolt;
  }
  DataPayload.resize(sizeof(Data));
  memcpy(DataPayload.data(),&Data_,DataPayload.size());
  SendPacket(DataPacket,DataPayload);
}

/* Sends byte buffer given meta data */
void TelemetryClient::SendPacket(PacketType_ Type, std::vector<uint8_t> &Buffer) {
  std::vector<uint8_t> TelemBuffer;
  TelemBuffer.resize(headerLength_ + Buffer.size() + checksumLength_);
  // header
  TelemBuffer[0] = header_[0];
  TelemBuffer[1] = header_[1];
  // data source
  TelemBuffer[2] = (uint8_t) Type;
  // data length
  TelemBuffer[3] = Buffer.size() & 0xff;
  TelemBuffer[4] = Buffer.size() >> 8;
  // payload
  std::memcpy(TelemBuffer.data()+headerLength_,Buffer.data(),Buffer.size());
  // checksum
  CalcChecksum((size_t)(Buffer.size()+headerLength_),TelemBuffer.data(),Checksum_);
  TelemBuffer[Buffer.size()+headerLength_] = Checksum_[0];
  TelemBuffer[Buffer.size()+headerLength_+1] = Checksum_[1];
  // write to UDP
  sendto(TelemetrySocket_,TelemBuffer.data(),TelemBuffer.size(),0,(struct sockaddr *)&TelemetryServer_,sizeof(TelemetryServer_));
}

/* Computes a two byte checksum. */
void TelemetryClient::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}

TelemetryServer::TelemetryServer() {
  TelemetrySocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  TelemetryServer_.sin_family = AF_INET;
  TelemetryServer_.sin_port = htons(TelemetryPort_);
  TelemetryServer_.sin_addr.s_addr = inet_addr("127.0.0.1");
  if (bind(TelemetrySocket_, (struct sockaddr *) &TelemetryServer_,sizeof(TelemetryServer_)) < 0) {
    throw std::runtime_error("Error binding to UDP port.");
  }
  Buffer.resize(kUartBufferMaxSize);
}

void TelemetryServer::ReceivePacket() {
  PacketType_ Type;
  std::vector<uint8_t> Payload;
  ssize_t MessageSize = recv(TelemetrySocket_,Buffer.data(),Buffer.size(),0);
  if (MessageSize > 0) {
    for (size_t i=0; i < MessageSize; i++) {
      if (ParseMessage(Buffer[i],&Type,&Payload)) {
        if (Type == UartPacket) {
          Uart.resize(Payload.size());
          for (size_t j=0; j < Payload.size(); j++) {
            Uart[j] = (char) Payload[j];
          }
          rxUart = true;
        }
        if (Type == BaudPacket) {
          memcpy(&Baud,Payload.data(),sizeof(Baud));
          rxBaud = true;
        }
        if (Type == DataPacket) {
          memcpy(&Data_,Payload.data(),sizeof(Data_));
          update(Data_);
        }
      }
    }
  }
  if ((rxUart)&&(rxBaud)&&(!uartLatch)) {
    if ((FileDesc_=open(Uart.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK))<0) {
      throw std::runtime_error(std::string("ERROR")+std::string(": UART failed to open."));
    } else {
      std::cout << "Starting telemetry UART" << std::endl;
      std::cout << "UART: " << Uart << std::endl;
      std::cout << "Baud: " << Baud << std::endl;
    }
    struct termios Options;
    tcgetattr(FileDesc_,&Options);
    // Options.c_cflag = (speed_t)Baud | CS8 | CREAD | CLOCAL;// WRONG WAY TO DO BAUD!!!!
    if ( Baud == 115200 ) {
        Options.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
    } else {
        printf("FIXME: bauds other than 115200 not supported in code\n");
    }
    Options.c_iflag = IGNPAR;
    Options.c_oflag = 0;
    Options.c_lflag = 0;
    Options.c_cc[VTIME] = 0;
    Options.c_cc[VMIN] = 0;
    tcflush(FileDesc_,TCIFLUSH);
    tcsetattr(FileDesc_,TCSANOW,&Options);
    fcntl(FileDesc_,F_SETFL,O_NONBLOCK);
    uartLatch = true;
  }
}

bool TelemetryServer::ParseMessage(uint8_t byte,PacketType_ *message,std::vector<uint8_t> *Payload) {
    RxByte_ = byte;
    // header
    if (ParserState_ < 2) {
      if (RxByte_ == header_[ParserState_]) {
        Buffer_[ParserState_] = RxByte_;
        ParserState_++;
      }
    // length
    } else if (ParserState_ == 3) {
      LengthBuffer_[0] = RxByte_;
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    } else if (ParserState_ == 4) {
      LengthBuffer_[1] = RxByte_;
      Length_ = ((uint16_t)LengthBuffer_[1] << 8) | LengthBuffer_[0];
      if (Length_ > (kUartBufferMaxSize-headerLength_-checksumLength_)) {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    // message ID and payload
    } else if (ParserState_ < (Length_ + headerLength_)) {
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    // checksum 0
    } else if (ParserState_ == (Length_ + headerLength_)) {
      CalcChecksum(Length_ + headerLength_,Buffer_,Checksum_);
      if (RxByte_ == Checksum_[0]) {
        ParserState_++;
      } else {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
    // checksum 1
    } else if (ParserState_ == (Length_ + headerLength_ + 1)) {
      if (RxByte_ == Checksum_[1]) {
        // message ID
        *message = (PacketType_) Buffer_[2];
        // payload size
        Payload->resize(Length_);
        // payload
        std::memcpy(Payload->data(),Buffer_+headerLength_,Length_);
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return true;
      } else {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
    }
  return false;
}

/* Computes a two byte checksum. */
void TelemetryServer::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}

void TelemetryServer :: generate_cksum(uint8_t id, uint8_t size, uint8_t * buf, uint8_t & cksum0, uint8_t & cksum1)
{
   cksum0 = 0;
   cksum1 = 0;

   cksum0 += id;
   cksum1 += cksum0;

   cksum0 += size;
   cksum1 += cksum0;

   for (int i = 0; i < size; i++)
   {
      cksum0 += buf[i];
      cksum1 += cksum0;
   }

}
void TelemetryServer :: send_packet(uint8_t * package, uint8_t IDnum, uint8_t size)
{
   uint8_t buf[4];
   uint8_t checksum0;
   uint8_t checksum1;

   generate_cksum(IDnum, size, package, checksum0, checksum1);

   buf[0] = 147;
   buf[1] = 224;
   buf[2] = IDnum;
   buf[3] = size;

   write(FileDesc_, buf, 4);
   write(FileDesc_, package, size);

   buf[0] = checksum0;
   buf[1] = checksum1;

   write(FileDesc_, buf, 2);
}

   void TelemetryServer :: update(const Data &DataRef)
   {

   count = count+1;

   if (count < 10) {
      return;
   }

   count = 0;

   gpsPacket gps;
   gps.index = 0;
   gps.timestamp = DataRef.Time.Time_us/1000000.0;
   gps.lat_deg = DataRef.Gps.Lat*(180/M_PI);//pi?
   gps.long_deg = DataRef.Gps.Lon*(180/M_PI);
   gps.alt_m = DataRef.Gps.Alt;
   gps.vn_ms = DataRef.Gps.Vn*100;
   gps.ve_ms = DataRef.Gps.Ve*100;
   gps.vd_ms = DataRef.Gps.Vd*100;
   gps.unix_time_sec = DataRef.Time.Time_us/1000000.0;
   gps.satellites = DataRef.Gps.NumberSatellites;
   gps.horiz_accuracy_m = DataRef.Gps.HAcc*100;
   gps.vert_accuracy_m = DataRef.Gps.VAcc*100;
   gps.pdop = DataRef.Gps.pDOP*100;
   gps.fixType = DataRef.Gps.Fix;


   airPacket air;
   air.index = 0;
   air.timestamp = DataRef.Time.Time_us/1000000.0;
   air.pressure_mbar = DataRef.StaticPress.Pressure_Pa/10.0;
   air.temp_degC = DataRef.StaticPress.Temperature_C*100;
   const double mps2kt = 1.9438444924406046432;
   air.airspeed_smoothed_kt = DataRef.Airspeed.Airspeed_ms * mps2kt * 100;
   air.altitude_smoothed_m = DataRef.Alt.Alt_m;
   air.altitude_true_m = DataRef.Alt.Alt_m;
   air.pressure_vertical_speed_fps=1;//maybe find?
   air.wind_dir_deg=1;//no
   air.wind_speed_kt=1;//no
   air.pitot_scale_factor=1;//no
   air.status = 0;

   pilotPacket pilot;
   pilot.index = 0;
   pilot.time = DataRef.Time.Time_us/1000000.0;
   pilot.chan[0] = DataRef.Sbus.Channels[0]*20000;
   pilot.chan[1] = DataRef.Sbus.Channels[1]*20000;
   pilot.chan[2] = DataRef.Sbus.Channels[2]*20000;
   pilot.chan[3] = DataRef.Sbus.Channels[3]*20000;
   pilot.chan[4] = DataRef.Sbus.Channels[4]*20000;
   pilot.chan[5] = DataRef.Sbus.Channels[5]*20000;
   pilot.chan[6] = DataRef.Sbus.Channels[6]*20000;
   pilot.chan[7] = 0;
   pilot.status = 0;


   ImunodePacket imu;
   imu.index = 0;
   imu.imu_timestamp = DataRef.Time.Time_us/1000000.0;
   imu.p_rad_sec = DataRef.Imu.Gx;
   imu.q_rad_sec = DataRef.Imu.Gy;
   imu.r_rad_sec = DataRef.Imu.Gz;
   imu.ax_mps_sec = DataRef.Imu.Ax;
   imu.ay_mps_sec = DataRef.Imu.Ay;
   imu.az_mps_sec = DataRef.Imu.Az;
   imu.hx = DataRef.Imu.Hx;
   imu.hy = DataRef.Imu.Hy;
   imu.hz = DataRef.Imu.Hz;
   imu.temp_C = DataRef.Imu.Temperature_C;
   imu.status = 0;


   filterPacket filter;
   filter.index = 0;
   filter.timestamp = DataRef.Time.Time_us/1000000.0;
   filter.latitude_deg = DataRef.Attitude.Lat*(180/M_PI);
   filter.longitude_deg = DataRef.Attitude.Lon*(180/M_PI);
   filter.altitude_m = DataRef.Attitude.Alt;
   filter.vn_ms = DataRef.Attitude.Vn*100;
   filter.ve_ms = DataRef.Attitude.Ve*100;
   filter.vd_ms = DataRef.Attitude.Vd*100;
   filter.roll_deg = DataRef.Attitude.Roll*10*(180/M_PI);
   filter.pitch_deg = DataRef.Attitude.Pitch*10*(180/M_PI);
   filter.heading_deg = DataRef.Attitude.Heading*10*(180/M_PI);
   filter.p_bias = DataRef.Attitude.Gxb*1000;
   filter.q_bias = DataRef.Attitude.Gyb*1000;
   filter.r_bias = DataRef.Attitude.Gzb*1000;
   filter.ax_bias = DataRef.Attitude.Axb*1000;
   filter.ay_bias = DataRef.Attitude.Ayb*1000;
   filter.az_bias = DataRef.Attitude.Azb*1000;
   filter.sequence_num = 1;
   filter.status = 0;


   ap_status ap;
   ap.index;
   ap.frame_time;
   ap.flags; //?
   ap.groundtrack_deg;
   ap.roll_deg;
   ap.Target_msl_ft;
   ap.ground_m;
   ap.pitch_deg;
   ap.airspeed_kt;
   ap.flight_timer;
   ap.target_waypoint_idx;
   if( sizeof(filter.latitude_deg) != 0 && num2 !=0) {
       ap.wp_lon = DataRef.Attitude.Lon*(180/M_PI);
       ap.wp_lat = DataRef.Attitude.Lat*(180/M_PI);;
       num2 = 0;
   }
   ap.wp_index;
   ap.routesize = 1;
   ap.sequence_num;




 /*  ap.timestamp = fmuData.Time_us/1000000.0;
   ap.master_switch;
   ap.pilot_pass_through;
   ap.groundtrack_deg;
   ap.roll_deg;
   ap.altitude_msl_ft;
   ap.altitude_ground_m;
   ap.pitch_deg;
   ap.airspeed_kt;
   ap.flight_timer;
   ap.target_waypoint_idx;
if (filter.latitude_deg != 0 && num2 != 0) {
   ap.longitude_deg = navOut.LLA[0]*(180/M_PI);
   ap.latitude_deg = navOut.LLA[0]*(180/M_PI);
   num2 = 0;
}
   ap.route_size;
   ap.sequence_num;
   ap.index = 65535;


   numactPacket numact1;
   numact1.index;
   numact1.timestamp = fmuData.Time_us;
   numact1.aileron;
   numact1.elevator;
   numact1.throttle;
   numact1.rudder;
   numact1.channel5;
   numact1.flaps;
   numact1.channel7;
   numact1.channel8;
   numact1.status;
 */
   
   healthPacket health;
   health.index;
   health.frame_time = DataRef.Time.Time_us/1000000.0;
   health.system_load_avg;
   health.board_vcc;
   health.extern_volts;
   health.extern_cell_volts = (int)(DataRef.Power.MinCellVolt * 1000.0);
   health.extern_amps;
   health.dekamah;

   uint8_t IDnum;
   uint8_t size;
//ap
   IDnum = 32;
   size = sizeof(ap);
   send_packet((uint8_t *)(&ap), IDnum, size);
//GPS BdddfhhhdBHHHB
   IDnum = 26;
   size = sizeof(gps);
   send_packet((uint8_t *)(&gps), IDnum, size);
//airdata BdHhhffhHBBB
   IDnum = 18;
   size = sizeof(air);
   send_packet((uint8_t *)(&air), IDnum, size);
//pilotcontrol BdhhhhhhhhB
   IDnum = 20;
   size = sizeof(pilot);
   send_packet((uint8_t *)(&pilot), IDnum, size);
//imudata BdfffffffffhB
   IDnum = 17;
   size = sizeof(imu);
   send_packet((uint8_t *)(&imu), IDnum, size);
//filterdata BdddfhhhhhhhhhhhhBB
   IDnum = 31;
   size = sizeof(filter);
   send_packet((uint8_t *)(&filter), IDnum, size);
/*
//actdata BdhhHhhhhhB
   size = sizeof(numact1);
   IDnum = 21;
   send_packet((uint8_t *)(&numact1), IDnum, size);
*/
//health BfHHHHHH
   size = sizeof(health);
   IDnum = 41;
   send_packet((uint8_t *)(&health), IDnum, size);
   };
