#include <iostream>
using std::cout;
using std::endl;

#include "telemetry.h"

#include "aura_messages.h"

static const double mps2kt = 1.9438444924406046432;

/* Opens a socket for telemetry */
TelemetryClient::TelemetryClient() {
  TelemetrySocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  TelemetryServer_.sin_family = AF_INET;
  TelemetryServer_.sin_port = htons(TelemetryPort_);
  TelemetryServer_.sin_addr.s_addr = inet_addr("127.0.0.1");
}

void TelemetryClient::Configure(const rapidjson::Value& Config) {
  if (Config.HasMember("Uart")) {
    Uart = Config["Uart"].GetString();
    SendPacket(UartPacket, (uint8_t *)(Uart.c_str()), Uart.length());
  } else {
    throw std::runtime_error(std::string("ERROR")+_RootPath+std::string(": Uart not specified in configuration."));
  }
  if (Config.HasMember("Baud")) {
    Baud = Config["Baud"].GetUint();
    SendPacket(BaudPacket, (uint8_t *)(&Baud), sizeof(Baud));
  } else {
    throw std::runtime_error(std::string("ERROR")+_RootPath+std::string(": Baud not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    Nodes_.Time.Time_us = deftree.getElement(Config["Time"].GetString());
    useTime = true;
  }
  if (Config.HasMember("Static-Pressure")) {
    std::string Sensor = Config["Static-Pressure"].GetString();
    Nodes_.StaticPress.Pressure_Pa = deftree.getElement(Sensor+"/Pressure_Pa");
    Nodes_.StaticPress.Temperature_C = deftree.getElement(Sensor+"/Temperature_C");
    useStaticPressure = true;
  }
  if (Config.HasMember("Airspeed")) {
    Nodes_.Airspeed.Airspeed_ms = deftree.getElement(Config["Airspeed"].GetString());
    useAirspeed = true;
  }
  if (Config.HasMember("Altitude")) {
    Nodes_.Alt.Alt_m = deftree.getElement(Config["Altitude"].GetString());
    useAlt = true;
  }
  if (Config.HasMember("Filter")) {
    std::string Sensor = Config["Filter"].GetString();
    Nodes_.Attitude.Ax = deftree.getElement(Sensor+"/AccelX_mss");
    Nodes_.Attitude.Axb = deftree.getElement(Sensor+"/AccelXBias_mss");
    Nodes_.Attitude.Ay = deftree.getElement(Sensor+"/AccelY_mss");
    Nodes_.Attitude.Ayb = deftree.getElement(Sensor+"/AccelYBias_mss");
    Nodes_.Attitude.Az = deftree.getElement(Sensor+"/AccelZ_mss");
    Nodes_.Attitude.Azb = deftree.getElement(Sensor+"/AccelZBias_mss");
    Nodes_.Attitude.Gx = deftree.getElement(Sensor+"/GyroX_rads");
    Nodes_.Attitude.Gxb = deftree.getElement(Sensor+"/GyroXBias_rads");
    Nodes_.Attitude.Gy = deftree.getElement(Sensor+"/GyroY_rads");
    Nodes_.Attitude.Gyb = deftree.getElement(Sensor+"/GyroYBias_rads");
    Nodes_.Attitude.Gz = deftree.getElement(Sensor+"/GyroZ_rads");
    Nodes_.Attitude.Gzb = deftree.getElement(Sensor+"/GyroZBias_rads");
    Nodes_.Attitude.Pitch = deftree.getElement(Sensor+"/Pitch_rad");
    Nodes_.Attitude.Roll = deftree.getElement(Sensor+"/Roll_rad");
    Nodes_.Attitude.Yaw = deftree.getElement(Sensor+"/Yaw_rad");
    Nodes_.Attitude.Heading = deftree.getElement(Sensor+"/Heading_rad");
    Nodes_.Attitude.Track = deftree.getElement(Sensor+"/Track_rad");
    Nodes_.Attitude.Lon = deftree.getElement(Sensor+"/Longitude_rad");
    Nodes_.Attitude.Lat = deftree.getElement(Sensor+"/Latitude_rad");
    Nodes_.Attitude.Alt = deftree.getElement(Sensor+"/Altitude_m");
    Nodes_.Attitude.Vn = deftree.getElement(Sensor+"/NorthVelocity_ms");
    Nodes_.Attitude.Ve = deftree.getElement(Sensor+"/EastVelocity_ms");
    Nodes_.Attitude.Vd = deftree.getElement(Sensor+"/DownVelocity_ms");
    useAttitude = true;
  }
  if (Config.HasMember("Gps")) {
    std::string Sensor = Config["Gps"].GetString();
    Nodes_.Gps.Fix = deftree.getElement(Sensor+"/Fix");
    Nodes_.Gps.NumberSatellites = deftree.getElement(Sensor+"/NumberSatellites");
    Nodes_.Gps.TOW = deftree.getElement(Sensor+"/TOW");
    Nodes_.Gps.Year = deftree.getElement(Sensor+"/Year");
    Nodes_.Gps.Month = deftree.getElement(Sensor+"/Month");
    Nodes_.Gps.Day = deftree.getElement(Sensor+"/Day");
    Nodes_.Gps.Hour = deftree.getElement(Sensor+"/Hour");
    Nodes_.Gps.Min = deftree.getElement(Sensor+"/Minute");
    Nodes_.Gps.Sec = deftree.getElement(Sensor+"/Second");
    Nodes_.Gps.Lat = deftree.getElement(Sensor+"/Latitude_rad");
    Nodes_.Gps.Lon = deftree.getElement(Sensor+"/Longitude_rad");
    Nodes_.Gps.Alt = deftree.getElement(Sensor+"/Altitude_m");
    Nodes_.Gps.Vn = deftree.getElement(Sensor+"/NorthVelocity_ms");
    Nodes_.Gps.Ve = deftree.getElement(Sensor+"/EastVelocity_ms");
    Nodes_.Gps.Vd = deftree.getElement(Sensor+"/DownVelocity_ms");
    Nodes_.Gps.HAcc = deftree.getElement(Sensor+"/HorizontalAccuracy_m");
    Nodes_.Gps.VAcc = deftree.getElement(Sensor+"/VerticalAccuracy_m");
    Nodes_.Gps.SAcc = deftree.getElement(Sensor+"/VelocityAccuracy_ms");
    Nodes_.Gps.pDOP = deftree.getElement(Sensor+"/pDOP");
    useGps = true;
  }
  if (Config.HasMember("Imu")) {
    std::string Sensor = Config["Imu"].GetString();
    Nodes_.Imu.Ax = deftree.getElement(Sensor+"/AccelX_mss");
    Nodes_.Imu.Ay = deftree.getElement(Sensor+"/AccelY_mss");
    Nodes_.Imu.Az = deftree.getElement(Sensor+"/AccelZ_mss");
    Nodes_.Imu.Gx = deftree.getElement(Sensor+"/GyroX_rads");
    Nodes_.Imu.Gy = deftree.getElement(Sensor+"/GyroY_rads");
    Nodes_.Imu.Gz = deftree.getElement(Sensor+"/GyroZ_rads");
    Nodes_.Imu.Temperature_C = deftree.getElement(Sensor+"/Temperature_C");
    useImu = true;
  }
  if (Config.HasMember("Sbus")) {
    std::string Sensor = Config["Sbus"].GetString();
    Nodes_.Sbus.FailSafe = deftree.getElement(Sensor+"/FailSafe");
    Nodes_.Sbus.LostFrames = deftree.getElement(Sensor+"/LostFrames");
    for (size_t j=0; j < 16; j++) {
      Nodes_.Sbus.Channels[j] = deftree.getElement(Sensor+"/Channels/"+std::to_string(j));
    }
    useSbus = true;
  }
  if (Config.HasMember("Power")) {
    std::string Power = Config["Power"].GetString();
    Nodes_.Power.MinCellVolt = deftree.getElement(Power+"/MinCellVolt_V");
    usePower = true;
  }
}

void TelemetryClient::Send() {
  count++;		// counts and updates assumes a 50hz loop rate

  float timestamp_sec = 0.0;
  if (useTime) {
    timestamp_sec = Nodes_.Time.Time_us->getLong() / 1000000.0;
  }
  // cout << count << " " << timestamp_sec << endl;
  
  if ( useGps && (count+0)%10 == 0 ) { // 5hz
    message_gps_v4_t gps;
    gps.index = 0;
    gps.timestamp_sec = timestamp_sec;
    gps.latitude_deg = Nodes_.Gps.Lat->getDouble() * (180/M_PI);
    gps.longitude_deg = Nodes_.Gps.Lon->getDouble() * (180/M_PI);
    gps.altitude_m = Nodes_.Gps.Alt->getFloat();
    gps.vn_ms = Nodes_.Gps.Vn->getFloat();
    gps.ve_ms = Nodes_.Gps.Ve->getFloat();
    gps.vd_ms = Nodes_.Gps.Vd->getFloat();
    gps.unixtime_sec = timestamp_sec;
    gps.satellites = Nodes_.Gps.NumberSatellites->getInt();
    gps.horiz_accuracy_m = Nodes_.Gps.HAcc->getFloat();
    gps.vert_accuracy_m = Nodes_.Gps.VAcc->getFloat();
    gps.pdop = Nodes_.Gps.pDOP->getFloat();
    gps.fix_type = Nodes_.Gps.Fix->getInt();
    gps.pack();
    SendPacket(gps.id, gps.payload, gps.len);
  }
  
  if ( (count+1)%10 == 0 ) {
    message_airdata_v7_t air;
    air.index = 0;
    air.timestamp_sec = timestamp_sec;
    if ( useStaticPressure ) {
      air.pressure_mbar = Nodes_.StaticPress.Pressure_Pa->getFloat();
      air.temp_C = Nodes_.StaticPress.Temperature_C->getFloat();
    } else {
      air.pressure_mbar = 0.0;
      air.temp_C = 0.0;
    }
    if ( useAirspeed ) {
      air.airspeed_smoothed_kt = Nodes_.Airspeed.Airspeed_ms->getFloat() * mps2kt;
    } else {
      air.airspeed_smoothed_kt = 0.0;
    }
    if ( useAlt ) {
      air.altitude_smoothed_m = Nodes_.Alt.Alt_m->getFloat();
      air.altitude_true_m = Nodes_.Alt.Alt_m->getFloat();
    } else {
      air.altitude_smoothed_m = 0.0;
      air.altitude_true_m = 0.0;
    }
    air.pressure_vertical_speed_fps = 0.0; // not used
    air.wind_dir_deg = 0.0; //no
    air.wind_speed_kt = 0.0; //no
    air.pitot_scale_factor = 1; //no
    air.status = 0;
    air.pack();
    SendPacket(air.id, air.payload, air.len);
  }
  
  if ( useImu && (count+2)%10 == 0) { // 5hz
    message_imu_v4_t imu;
    imu.index = 0;
    imu.timestamp_sec = timestamp_sec;
    imu.p_rad_sec = Nodes_.Imu.Gx->getFloat();
    imu.q_rad_sec = Nodes_.Imu.Gy->getFloat();
    imu.r_rad_sec = Nodes_.Imu.Gz->getFloat();
    imu.ax_mps_sec = Nodes_.Imu.Ax->getFloat();
    imu.ay_mps_sec = Nodes_.Imu.Ay->getFloat();
    imu.az_mps_sec = Nodes_.Imu.Az->getFloat();
    imu.hx = 0.0;
    imu.hy = 0.0;
    imu.hz = 0.0;
    imu.temp_C = Nodes_.Imu.Temperature_C->getFloat();
    imu.status = 0;
    imu.pack();
    SendPacket(imu.id, imu.payload, imu.len);
  }
  
  if ( useAttitude && (count+3)%5 == 0 ) { // 10hz
    message_filter_v4_t nav;
    nav.index = 0;
    nav.timestamp_sec = timestamp_sec;
    nav.latitude_deg = Nodes_.Attitude.Lat->getDouble() * (180/M_PI);
    nav.longitude_deg = Nodes_.Attitude.Lon->getDouble() * (180/M_PI);
    nav.altitude_m = Nodes_.Attitude.Alt->getFloat();
    nav.vn_ms = Nodes_.Attitude.Vn->getFloat();
    nav.ve_ms = Nodes_.Attitude.Ve->getFloat();
    nav.vd_ms = Nodes_.Attitude.Vd->getFloat();
    nav.roll_deg = Nodes_.Attitude.Roll->getFloat() * (180/M_PI);
    nav.pitch_deg = Nodes_.Attitude.Pitch->getFloat() * (180/M_PI);
    nav.yaw_deg = Nodes_.Attitude.Heading->getFloat() * (180/M_PI);
    nav.p_bias = Nodes_.Attitude.Gxb->getFloat();
    nav.q_bias = Nodes_.Attitude.Gyb->getFloat();
    nav.r_bias = Nodes_.Attitude.Gzb->getFloat();
    nav.ax_bias = Nodes_.Attitude.Axb->getFloat();
    nav.ay_bias = Nodes_.Attitude.Ayb->getFloat();
    nav.az_bias = Nodes_.Attitude.Azb->getFloat();
    nav.sequence_num = 0;
    nav.status = 0;
    nav.pack();
    SendPacket(nav.id, nav.payload, nav.len);
  }
  
  if ( useSbus && (count+4)%10 == 0 ) { // 5hz
    message_pilot_v3_t pilot;
    pilot.index = 0;
    pilot.timestamp_sec = timestamp_sec;
    for ( size_t j = 0; j < 8; j++ ) {
      pilot.channel[j] = Nodes_.Sbus.Channels[j]->getFloat();
    }
    pilot.status = 0;
    pilot.pack();
    SendPacket(pilot.id, pilot.payload, pilot.len);
  }
  
  if ( usePower && (count+5)%10 == 0 ) { // 5hz
    message_system_health_v5_t health;
    health.index;
    health.timestamp_sec = timestamp_sec;
    health.system_load_avg = 0.0;
    health.avionics_vcc = 0.0;
    health.main_vcc = 0.0;
    health.cell_vcc = Nodes_.Power.MinCellVolt->getFloat();
    health.main_amps = 0.0;
    health.total_mah = 0.0;
    health.pack();
    SendPacket(health.id, health.payload, health.len);
  }
}

/* Sends byte buffer given meta data */
void TelemetryClient::SendPacket(uint16_t pkt_id, uint8_t *Buffer, uint16_t len ) {
  // cout << "-> " << pkt_id << endl;
  std::vector<uint8_t> TelemBuffer;
  TelemBuffer.resize(sizeof(pkt_id) + len);
  // header
  *(TelemBuffer.data()) = pkt_id;
  // payload
  std::memcpy(TelemBuffer.data()+sizeof(pkt_id), Buffer, len);
  // write to UDP
  sendto(TelemetrySocket_,TelemBuffer.data(),TelemBuffer.size(),0,(struct sockaddr *)&TelemetryServer_,sizeof(TelemetryServer_));
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
  std::vector<uint8_t> Payload;
  ssize_t MessageSize = recv(TelemetrySocket_,Buffer.data(),Buffer.size(),0);
  if (MessageSize > 0) {
    uint16_t pkt_id = *(uint16_t *)(Buffer.data());
    uint16_t len = MessageSize - sizeof(pkt_id);
    uint8_t *payload = Buffer.data() + sizeof(pkt_id);
    if ( pkt_id == UartPacket ) {
      Uart = std::string((char *)payload, len);
      for (size_t j=0; j < Payload.size(); j++) {
	Uart[j] = (char) Payload[j];
      }
      rxUart = true;
    } else if ( pkt_id == BaudPacket ) {
      Baud = *(uint32_t *)payload;
      rxBaud = true;
    } else {
      send_packet(pkt_id, payload, len);
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

void TelemetryServer :: generate_cksum(uint8_t id, uint8_t size, uint8_t * buf, uint8_t & cksum0, uint8_t & cksum1)
{
  cksum0 = 0;
  cksum1 = 0;

  cksum0 += id;
  cksum1 += cksum0;

  cksum0 += size;
  cksum1 += cksum0;

  for (int i = 0; i < size; i++) {
    cksum0 += buf[i];
    cksum1 += cksum0;
  }
}

void TelemetryServer :: send_packet(uint8_t id, uint8_t *payload, uint8_t len)
{
  uint8_t buf[4];
  uint8_t checksum0;
  uint8_t checksum1;

  generate_cksum(id, len, payload, checksum0, checksum1);

  buf[0] = 147;
  buf[1] = 224;
  buf[2] = id;
  buf[3] = len;

  write(FileDesc_, buf, 4);
  write(FileDesc_, payload, len);

  buf[0] = checksum0;
  buf[1] = checksum1;

  write(FileDesc_, buf, 2);
}
