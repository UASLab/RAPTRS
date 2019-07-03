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
    TimeNodes.Time_us = deftree.getElement(Config["Time"].GetString());
    useTime = true;
  }
  if (Config.HasMember("Static-Pressure")) {
    std::string Sensor = Config["Static-Pressure"].GetString();
    StaticPressNodes.Pressure_Pa = deftree.getElement(Sensor+"/Pressure_Pa");
    StaticPressNodes.Temperature_C = deftree.getElement(Sensor+"/Temperature_C");
    useStaticPressure = true;
  }
  if (Config.HasMember("Airspeed")) {
    AirspeedNodes.Airspeed_ms = deftree.getElement(Config["Airspeed"].GetString());
    useAirspeed = true;
  }
  if (Config.HasMember("Altitude")) {
    AltNodes.Alt_m = deftree.getElement(Config["Altitude"].GetString());
    useAlt = true;
  }
  if (Config.HasMember("Filter")) {
    std::string Sensor = Config["Filter"].GetString();
    AttitudeNodes.Ax = deftree.getElement(Sensor+"/AccelX_mss");
    AttitudeNodes.Axb = deftree.getElement(Sensor+"/AccelXBias_mss");
    AttitudeNodes.Ay = deftree.getElement(Sensor+"/AccelY_mss");
    AttitudeNodes.Ayb = deftree.getElement(Sensor+"/AccelYBias_mss");
    AttitudeNodes.Az = deftree.getElement(Sensor+"/AccelZ_mss");
    AttitudeNodes.Azb = deftree.getElement(Sensor+"/AccelZBias_mss");
    AttitudeNodes.Gx = deftree.getElement(Sensor+"/GyroX_rads");
    AttitudeNodes.Gxb = deftree.getElement(Sensor+"/GyroXBias_rads");
    AttitudeNodes.Gy = deftree.getElement(Sensor+"/GyroY_rads");
    AttitudeNodes.Gyb = deftree.getElement(Sensor+"/GyroYBias_rads");
    AttitudeNodes.Gz = deftree.getElement(Sensor+"/GyroZ_rads");
    AttitudeNodes.Gzb = deftree.getElement(Sensor+"/GyroZBias_rads");
    AttitudeNodes.Pitch = deftree.getElement(Sensor+"/Pitch_rad");
    AttitudeNodes.Roll = deftree.getElement(Sensor+"/Roll_rad");
    AttitudeNodes.Yaw = deftree.getElement(Sensor+"/Yaw_rad");
    AttitudeNodes.Heading = deftree.getElement(Sensor+"/Heading_rad");
    AttitudeNodes.Track = deftree.getElement(Sensor+"/Track_rad");
    AttitudeNodes.Lon = deftree.getElement(Sensor+"/Longitude_rad");
    AttitudeNodes.Lat = deftree.getElement(Sensor+"/Latitude_rad");
    AttitudeNodes.Alt = deftree.getElement(Sensor+"/Altitude_m");
    AttitudeNodes.Vn = deftree.getElement(Sensor+"/NorthVelocity_ms");
    AttitudeNodes.Ve = deftree.getElement(Sensor+"/EastVelocity_ms");
    AttitudeNodes.Vd = deftree.getElement(Sensor+"/DownVelocity_ms");
    useAttitude = true;
  }
  if (Config.HasMember("Gps")) {
    std::string Sensor = Config["Gps"].GetString();
    GpsNodes.Fix = deftree.getElement(Sensor+"/Fix");
    GpsNodes.NumberSatellites = deftree.getElement(Sensor+"/NumberSatellites");
    GpsNodes.TOW = deftree.getElement(Sensor+"/TOW");
    GpsNodes.Year = deftree.getElement(Sensor+"/Year");
    GpsNodes.Month = deftree.getElement(Sensor+"/Month");
    GpsNodes.Day = deftree.getElement(Sensor+"/Day");
    GpsNodes.Hour = deftree.getElement(Sensor+"/Hour");
    GpsNodes.Min = deftree.getElement(Sensor+"/Minute");
    GpsNodes.Sec = deftree.getElement(Sensor+"/Second");
    GpsNodes.Lat = deftree.getElement(Sensor+"/Latitude_rad");
    GpsNodes.Lon = deftree.getElement(Sensor+"/Longitude_rad");
    GpsNodes.Alt = deftree.getElement(Sensor+"/Altitude_m");
    GpsNodes.Vn = deftree.getElement(Sensor+"/NorthVelocity_ms");
    GpsNodes.Ve = deftree.getElement(Sensor+"/EastVelocity_ms");
    GpsNodes.Vd = deftree.getElement(Sensor+"/DownVelocity_ms");
    GpsNodes.HAcc = deftree.getElement(Sensor+"/HorizontalAccuracy_m");
    GpsNodes.VAcc = deftree.getElement(Sensor+"/VerticalAccuracy_m");
    GpsNodes.SAcc = deftree.getElement(Sensor+"/VelocityAccuracy_ms");
    GpsNodes.pDOP = deftree.getElement(Sensor+"/pDOP");
    useGps = true;
  }
  if (Config.HasMember("Imu")) {
    std::string Sensor = Config["Imu"].GetString();
    ImuNodes.Ax = deftree.getElement(Sensor+"/AccelX_mss");
    ImuNodes.Ay = deftree.getElement(Sensor+"/AccelY_mss");
    ImuNodes.Az = deftree.getElement(Sensor+"/AccelZ_mss");
    ImuNodes.Gx = deftree.getElement(Sensor+"/GyroX_rads");
    ImuNodes.Gy = deftree.getElement(Sensor+"/GyroY_rads");
    ImuNodes.Gz = deftree.getElement(Sensor+"/GyroZ_rads");
    ImuNodes.Temperature_C = deftree.getElement(Sensor+"/Temperature_C");
    useImu = true;
  }
  if (Config.HasMember("Sbus")) {
    std::string Sensor = Config["Sbus"].GetString();
    SbusNodes.FailSafe = deftree.getElement(Sensor+"/FailSafe");
    SbusNodes.LostFrames = deftree.getElement(Sensor+"/LostFrames");
    for (size_t j=0; j < 16; j++) {
      SbusNodes.Channels[j] = deftree.getElement(Sensor+"/Channels/"+std::to_string(j));
    }
    useSbus = true;
  }
  if (Config.HasMember("Power")) {
    std::string Power = Config["Power"].GetString();
    PowerNodes.MinCellVolt = deftree.getElement(Power+"/MinCellVolt_V");
    usePower = true;
  }
}

void TelemetryClient::Send() {
  count++;		// counts and updates assumes a 50hz loop rate

  float timestamp_sec = 0.0;
  if ( useTime ) {
    timestamp_sec = TimeNodes.Time_us->getLong() / 1000000.0;
  }
  // cout << count << " " << timestamp_sec << endl;
  
  if ( useGps && (count+0)%10 == 0 ) { // 5hz
    message_gps_v4_t gps;
    gps.index = 0;
    gps.timestamp_sec = timestamp_sec;
    gps.latitude_deg = GpsNodes.Lat->getDouble() * (180/M_PI);
    gps.longitude_deg = GpsNodes.Lon->getDouble() * (180/M_PI);
    gps.altitude_m = GpsNodes.Alt->getFloat();
    gps.vn_ms = GpsNodes.Vn->getFloat();
    gps.ve_ms = GpsNodes.Ve->getFloat();
    gps.vd_ms = GpsNodes.Vd->getFloat();
    gps.unixtime_sec = timestamp_sec;
    gps.satellites = GpsNodes.NumberSatellites->getInt();
    gps.horiz_accuracy_m = GpsNodes.HAcc->getFloat();
    gps.vert_accuracy_m = GpsNodes.VAcc->getFloat();
    gps.pdop = GpsNodes.pDOP->getFloat();
    gps.fix_type = GpsNodes.Fix->getInt();
    gps.pack();
    SendPacket(gps.id, gps.payload, gps.len);
  }
  
  if ( (count+1)%10 == 0 ) {	// 5hz
    message_airdata_v7_t air;
    air.index = 0;
    air.timestamp_sec = timestamp_sec;
    if ( useStaticPressure ) {
      air.pressure_mbar = StaticPressNodes.Pressure_Pa->getFloat();
      air.temp_C = StaticPressNodes.Temperature_C->getFloat();
    } else {
      air.pressure_mbar = 0.0;
      air.temp_C = 0.0;
    }
    if ( useAirspeed ) {
      air.airspeed_smoothed_kt = AirspeedNodes.Airspeed_ms->getFloat() * mps2kt;
    } else {
      air.airspeed_smoothed_kt = 0.0;
    }
    if ( useAlt ) {
      air.altitude_smoothed_m = AltNodes.Alt_m->getFloat();
      air.altitude_true_m = AltNodes.Alt_m->getFloat();
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
    imu.p_rad_sec = ImuNodes.Gx->getFloat();
    imu.q_rad_sec = ImuNodes.Gy->getFloat();
    imu.r_rad_sec = ImuNodes.Gz->getFloat();
    imu.ax_mps_sec = ImuNodes.Ax->getFloat();
    imu.ay_mps_sec = ImuNodes.Ay->getFloat();
    imu.az_mps_sec = ImuNodes.Az->getFloat();
    imu.hx = 0.0;
    imu.hy = 0.0;
    imu.hz = 0.0;
    imu.temp_C = ImuNodes.Temperature_C->getFloat();
    imu.status = 0;
    imu.pack();
    SendPacket(imu.id, imu.payload, imu.len);
  }
  
  if ( useAttitude && (count+3)%5 == 0 ) { // 10hz
    message_filter_v4_t nav;
    nav.index = 0;
    nav.timestamp_sec = timestamp_sec;
    nav.latitude_deg = AttitudeNodes.Lat->getDouble() * (180/M_PI);
    nav.longitude_deg = AttitudeNodes.Lon->getDouble() * (180/M_PI);
    nav.altitude_m = AttitudeNodes.Alt->getFloat();
    nav.vn_ms = AttitudeNodes.Vn->getFloat();
    nav.ve_ms = AttitudeNodes.Ve->getFloat();
    nav.vd_ms = AttitudeNodes.Vd->getFloat();
    nav.roll_deg = AttitudeNodes.Roll->getFloat() * (180/M_PI);
    nav.pitch_deg = AttitudeNodes.Pitch->getFloat() * (180/M_PI);
    nav.yaw_deg = AttitudeNodes.Heading->getFloat() * (180/M_PI);
    nav.p_bias = AttitudeNodes.Gxb->getFloat();
    nav.q_bias = AttitudeNodes.Gyb->getFloat();
    nav.r_bias = AttitudeNodes.Gzb->getFloat();
    nav.ax_bias = AttitudeNodes.Axb->getFloat();
    nav.ay_bias = AttitudeNodes.Ayb->getFloat();
    nav.az_bias = AttitudeNodes.Azb->getFloat();
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
      pilot.channel[j] = SbusNodes.Channels[j]->getFloat();
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
    health.cell_vcc = PowerNodes.MinCellVolt->getFloat();
    health.main_amps = 0.0;
    health.total_mah = 0.0;
    health.pack();
    SendPacket(health.id, health.payload, health.len);
  }
}

/* Sends byte buffer given meta data */
void TelemetryClient::SendPacket(uint16_t pkt_id, uint8_t *buf, uint16_t len ) {
  // cout << "-> " << pkt_id << endl;
  std::vector<uint8_t> TelemBuffer;
  TelemBuffer.resize(sizeof(pkt_id) + len);
  // header
  *(TelemBuffer.data()) = pkt_id;
  // payload
  std::memcpy(TelemBuffer.data()+sizeof(pkt_id), buf, len);
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
