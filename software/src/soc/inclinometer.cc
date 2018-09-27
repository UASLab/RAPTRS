
#include "inclinometer.hxx"

Incline::Incline() {
  OpenPort();
}

/* Opens port to communicate with Inclinometer. */
void Incline::OpenPort() {
  std::cout << "Opening UART port with Incline...";
  if ((InclineFileDesc_=open(InclinePort,O_RDWR|O_NOCTTY|O_NONBLOCK))<0) {
    throw std::runtime_error("UART failed to open.");
  }
  struct termios Options;
  tcgetattr(InclineFileDesc_,&Options);
  Options.c_cflag = InclineBaud | CS8 | CREAD | CLOCAL;
  Options.c_iflag = IGNPAR;
  Options.c_oflag = 0;
  Options.c_lflag = 0;
  Options.c_cc[VTIME] = 0;
  Options.c_cc[VMIN] = 0;
  sleep(2);
  tcflush(InclineFileDesc_,TCIFLUSH);
  tcsetattr(InclineFileDesc_,TCSANOW,&Options);
  fcntl(InclineFileDesc_,F_SETFL,O_NONBLOCK);
  std::cout << "done!" << std::endl;
}


/* Set the Damping*/
bool Incline::SetDamping() {
  const uint8_t SendDamp[5] = {0, 198, 0, 200, 114}; // Set Damping to 200 ms

  int count = 0 ;
  if ((count = write(InclineFileDesc_, SendDamp, 5)) < 0) {
    throw std::runtime_error("UART failed to write.");
  }
    
  usleep(25000);
  
  uint8_t RxBuffer[2] = {}; // 1-byte status, 1-Byte checksum
  count = -1;
  int status = -1;
  if ((count = read(InclineFileDesc_, RxBuffer, 2)) > 0) {

    uint8_t status = RxBuffer[0];
    uint8_t checkshum = RxBuffer[1];
    
    if (status != 0) {
      std::cout << "Status: " << unsigned(status) << std::endl;
      throw std::runtime_error("Inclinometer Damping set failed.");
    }
    
  } else {
    throw std::runtime_error("UART failed to read.");
  }
  
}

/* Get sensor data from Inclinometer. */
bool Incline::GetAngle(InclineData *InclineDataPtr) {
 
  const uint8_t SendBuff[3] = {01,224,00}; // Read angle on Axis1
 
  uint16_t RxPayloadSize;
  uint8_t RxPayload[sizeof(InclineDataPtr->Angle_deg)];
  
  union{
    int32_t val;
    uint8_t b[4];
  } angle;
  
  int count = 0 ;
  if ((count = write(InclineFileDesc_, SendBuff, 3)) < 0) {
    throw std::runtime_error("UART failed to write.");
  }
  
  usleep(25000);
  
  uint8_t RxBuffer[5] = {}; // 4-byte float, 1-Byte checksum
  count = -1;
  if ((count = read(InclineFileDesc_, RxBuffer, 5)) > 0) {

    angle.b[3] = RxBuffer[0];
    angle.b[2] = RxBuffer[1];
    angle.b[1] = RxBuffer[2];
    angle.b[0] = RxBuffer[3];
    
    InclineDataPtr->Angle_deg = -(float)angle.val / 1000.0;
    
  } else {
    throw std::runtime_error("UART failed to read.");
  }
}


